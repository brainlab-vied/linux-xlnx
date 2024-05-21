// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Pengutronix, Michael Tretter <kernel@pengutronix.de>
 *
 * Convert NAL units between raw byte sequence payloads (RBSP) and C structs
 *
 * The conversion is defined in "ITU-T Rec. H.264 (04/2017) Advanced video
 * coding for generic audiovisual services". Decoder drivers may use the
 * parser to parse RBSP from encoded streams and configure the hardware, if
 * the hardware is not able to parse RBSP itself.  Encoder drivers may use the
 * generator to generate the RBSP for SPS/PPS nal units and add them to the
 * encoded stream if the hardware does not generate the units.
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/v4l2-controls.h>
#include <media/v4l2-ctrls.h>

#include <linux/device.h>
#include <linux/export.h>
#include <linux/log2.h>

#include "nal-h264.h"
#include "nal-rbsp.h"

/*
 * See Rec. ITU-T H.264 (04/2017) Table 7-1 - NAL unit type codes, syntax
 * element categories, and NAL unit type classes
 */
enum nal_unit_type {
	SEQUENCE_PARAMETER_SET = 7,
	PICTURE_PARAMETER_SET = 8,
	FILLER_DATA = 12,
};

static void nal_h264_write_start_code_prefix(struct rbsp *rbsp)
{
	u8 *p = rbsp->data + DIV_ROUND_UP(rbsp->pos, 8);
	int i = 4;

	if (DIV_ROUND_UP(rbsp->pos, 8) + i > rbsp->size) {
		rbsp->error = -EINVAL;
		return;
	}

	p[0] = 0x00;
	p[1] = 0x00;
	p[2] = 0x00;
	p[3] = 0x01;

	rbsp->pos += i * 8;
}

static void nal_h264_read_start_code_prefix(struct rbsp *rbsp)
{
	u8 *p = rbsp->data + DIV_ROUND_UP(rbsp->pos, 8);
	int i = 4;

	if (DIV_ROUND_UP(rbsp->pos, 8) + i > rbsp->size) {
		rbsp->error = -EINVAL;
		return;
	}

	if (p[0] != 0x00 || p[1] != 0x00 || p[2] != 0x00 || p[3] != 0x01) {
		rbsp->error = -EINVAL;
		return;
	}

	rbsp->pos += i * 8;
}

static void nal_h264_write_filler_data(struct rbsp *rbsp)
{
	u8 *p = rbsp->data + DIV_ROUND_UP(rbsp->pos, 8);
	int i;

	/* Keep 1 byte extra for terminating the NAL unit */
	i = rbsp->size - DIV_ROUND_UP(rbsp->pos, 8) - 1;
	memset(p, 0xff, i);
	rbsp->pos += i * 8;
}

static void nal_h264_read_filler_data(struct rbsp *rbsp)
{
	u8 *p = rbsp->data + DIV_ROUND_UP(rbsp->pos, 8);

	while (*p == 0xff) {
		if (DIV_ROUND_UP(rbsp->pos, 8) > rbsp->size) {
			rbsp->error = -EINVAL;
			return;
		}

		p++;
		rbsp->pos += 8;
	}
}

static void nal_h264_rbsp_hrd_parameters(struct rbsp *rbsp,
					 struct nal_h264_hrd_parameters *hrd)
{
	unsigned int i;

	if (!hrd) {
		rbsp->error = -EINVAL;
		return;
	}

	rbsp_uev(rbsp, &hrd->cpb_cnt_minus1);
	rbsp_bits(rbsp, 4, &hrd->bit_rate_scale);
	rbsp_bits(rbsp, 4, &hrd->cpb_size_scale);

	for (i = 0; i <= hrd->cpb_cnt_minus1; i++) {
		rbsp_uev(rbsp, &hrd->bit_rate_value_minus1[i]);
		rbsp_uev(rbsp, &hrd->cpb_size_value_minus1[i]);
		rbsp_bit(rbsp, &hrd->cbr_flag[i]);
	}

	rbsp_bits(rbsp, 5, &hrd->initial_cpb_removal_delay_length_minus1);
	rbsp_bits(rbsp, 5, &hrd->cpb_removal_delay_length_minus1);
	rbsp_bits(rbsp, 5, &hrd->dpb_output_delay_length_minus1);
	rbsp_bits(rbsp, 5, &hrd->time_offset_length);
}

static void nal_h264_rbsp_vui_parameters(struct rbsp *rbsp,
					 struct nal_h264_vui_parameters *vui)
{
	if (!vui) {
		rbsp->error = -EINVAL;
		return;
	}

	rbsp_bit(rbsp, &vui->aspect_ratio_info_present_flag);
	if (vui->aspect_ratio_info_present_flag) {
		rbsp_bits(rbsp, 8, &vui->aspect_ratio_idc);
		if (vui->aspect_ratio_idc == 255) {
			rbsp_bits(rbsp, 16, &vui->sar_width);
			rbsp_bits(rbsp, 16, &vui->sar_height);
		}
	}

	rbsp_bit(rbsp, &vui->overscan_info_present_flag);
	if (vui->overscan_info_present_flag)
		rbsp_bit(rbsp, &vui->overscan_appropriate_flag);

	rbsp_bit(rbsp, &vui->video_signal_type_present_flag);
	if (vui->video_signal_type_present_flag) {
		rbsp_bits(rbsp, 3, &vui->video_format);
		rbsp_bit(rbsp, &vui->video_full_range_flag);

		rbsp_bit(rbsp, &vui->colour_description_present_flag);
		if (vui->colour_description_present_flag) {
			rbsp_bits(rbsp, 8, &vui->colour_primaries);
			rbsp_bits(rbsp, 8, &vui->transfer_characteristics);
			rbsp_bits(rbsp, 8, &vui->matrix_coefficients);
		}
	}

	rbsp_bit(rbsp, &vui->chroma_loc_info_present_flag);
	if (vui->chroma_loc_info_present_flag) {
		rbsp_uev(rbsp, &vui->chroma_sample_loc_type_top_field);
		rbsp_uev(rbsp, &vui->chroma_sample_loc_type_bottom_field);
	}

	rbsp_bit(rbsp, &vui->timing_info_present_flag);
	if (vui->timing_info_present_flag) {
		rbsp_bits(rbsp, 32, &vui->num_units_in_tick);
		rbsp_bits(rbsp, 32, &vui->time_scale);
		rbsp_bit(rbsp, &vui->fixed_frame_rate_flag);
	}

	rbsp_bit(rbsp, &vui->nal_hrd_parameters_present_flag);
	if (vui->nal_hrd_parameters_present_flag)
		nal_h264_rbsp_hrd_parameters(rbsp, &vui->nal_hrd_parameters);

	rbsp_bit(rbsp, &vui->vcl_hrd_parameters_present_flag);
	if (vui->vcl_hrd_parameters_present_flag)
		nal_h264_rbsp_hrd_parameters(rbsp, &vui->vcl_hrd_parameters);

	if (vui->nal_hrd_parameters_present_flag ||
	    vui->vcl_hrd_parameters_present_flag)
		rbsp_bit(rbsp, &vui->low_delay_hrd_flag);

	rbsp_bit(rbsp, &vui->pic_struct_present_flag);

	rbsp_bit(rbsp, &vui->bitstream_restriction_flag);
	if (vui->bitstream_restriction_flag) {
		rbsp_bit(rbsp, &vui->motion_vectors_over_pic_boundaries_flag);
		rbsp_uev(rbsp, &vui->max_bytes_per_pic_denom);
		rbsp_uev(rbsp, &vui->max_bits_per_mb_denom);
		rbsp_uev(rbsp, &vui->log2_max_mv_length_horizontal);
		rbsp_uev(rbsp, &vui->log21_max_mv_length_vertical);
		rbsp_uev(rbsp, &vui->max_num_reorder_frames);
		rbsp_uev(rbsp, &vui->max_dec_frame_buffering);
	}
}

static void nal_h264_rbsp_sps(struct rbsp *rbsp, struct nal_h264_sps *sps)
{
	unsigned int i;

	if (!sps) {
		rbsp->error = -EINVAL;
		return;
	}

	rbsp_bits(rbsp, 8, &sps->profile_idc);
	rbsp_bit(rbsp, &sps->constraint_set0_flag);
	rbsp_bit(rbsp, &sps->constraint_set1_flag);
	rbsp_bit(rbsp, &sps->constraint_set2_flag);
	rbsp_bit(rbsp, &sps->constraint_set3_flag);
	rbsp_bit(rbsp, &sps->constraint_set4_flag);
	rbsp_bit(rbsp, &sps->constraint_set5_flag);
	rbsp_bits(rbsp, 2, &sps->reserved_zero_2bits);
	rbsp_bits(rbsp, 8, &sps->level_idc);

	rbsp_uev(rbsp, &sps->seq_parameter_set_id);

	if (sps->profile_idc == 100 || sps->profile_idc == 110 ||
	    sps->profile_idc == 122 || sps->profile_idc == 244 ||
	    sps->profile_idc == 44 || sps->profile_idc == 83 ||
	    sps->profile_idc == 86 || sps->profile_idc == 118 ||
	    sps->profile_idc == 128 || sps->profile_idc == 138 ||
	    sps->profile_idc == 139 || sps->profile_idc == 134 ||
	    sps->profile_idc == 135) {
		rbsp_uev(rbsp, &sps->chroma_format_idc);

		if (sps->chroma_format_idc == 3)
			rbsp_bit(rbsp, &sps->separate_colour_plane_flag);
		rbsp_uev(rbsp, &sps->bit_depth_luma_minus8);
		rbsp_uev(rbsp, &sps->bit_depth_chroma_minus8);
		rbsp_bit(rbsp, &sps->qpprime_y_zero_transform_bypass_flag);
		rbsp_bit(rbsp, &sps->seq_scaling_matrix_present_flag);
		if (sps->seq_scaling_matrix_present_flag)
			rbsp->error = -EINVAL;
	}

	rbsp_uev(rbsp, &sps->log2_max_frame_num_minus4);

	rbsp_uev(rbsp, &sps->pic_order_cnt_type);
	switch (sps->pic_order_cnt_type) {
	case 0:
		rbsp_uev(rbsp, &sps->log2_max_pic_order_cnt_lsb_minus4);
		break;
	case 1:
		rbsp_bit(rbsp, &sps->delta_pic_order_always_zero_flag);
		rbsp_sev(rbsp, &sps->offset_for_non_ref_pic);
		rbsp_sev(rbsp, &sps->offset_for_top_to_bottom_field);

		rbsp_uev(rbsp, &sps->num_ref_frames_in_pic_order_cnt_cycle);
		for (i = 0; i < sps->num_ref_frames_in_pic_order_cnt_cycle; i++)
			rbsp_sev(rbsp, &sps->offset_for_ref_frame[i]);
		break;
	default:
		rbsp->error = -EINVAL;
		break;
	}

	rbsp_uev(rbsp, &sps->max_num_ref_frames);
	rbsp_bit(rbsp, &sps->gaps_in_frame_num_value_allowed_flag);
	rbsp_uev(rbsp, &sps->pic_width_in_mbs_minus1);
	rbsp_uev(rbsp, &sps->pic_height_in_map_units_minus1);

	rbsp_bit(rbsp, &sps->frame_mbs_only_flag);
	if (!sps->frame_mbs_only_flag)
		rbsp_bit(rbsp, &sps->mb_adaptive_frame_field_flag);

	rbsp_bit(rbsp, &sps->direct_8x8_inference_flag);

	rbsp_bit(rbsp, &sps->frame_cropping_flag);
	if (sps->frame_cropping_flag) {
		rbsp_uev(rbsp, &sps->crop_left);
		rbsp_uev(rbsp, &sps->crop_right);
		rbsp_uev(rbsp, &sps->crop_top);
		rbsp_uev(rbsp, &sps->crop_bottom);
	}

	rbsp_bit(rbsp, &sps->vui_parameters_present_flag);
	if (sps->vui_parameters_present_flag)
		nal_h264_rbsp_vui_parameters(rbsp, &sps->vui);
}

static void nal_h264_rbsp_pps(struct rbsp *rbsp, struct nal_h264_pps *pps)
{
	int i;

	rbsp_uev(rbsp, &pps->pic_parameter_set_id);
	rbsp_uev(rbsp, &pps->seq_parameter_set_id);
	rbsp_bit(rbsp, &pps->entropy_coding_mode_flag);
	rbsp_bit(rbsp, &pps->bottom_field_pic_order_in_frame_present_flag);
	rbsp_uev(rbsp, &pps->num_slice_groups_minus1);
	if (pps->num_slice_groups_minus1 > 0) {
		rbsp_uev(rbsp, &pps->slice_group_map_type);
		switch (pps->slice_group_map_type) {
		case 0:
			for (i = 0; i < pps->num_slice_groups_minus1; i++)
				rbsp_uev(rbsp, &pps->run_length_minus1[i]);
			break;
		case 2:
			for (i = 0; i < pps->num_slice_groups_minus1; i++) {
				rbsp_uev(rbsp, &pps->top_left[i]);
				rbsp_uev(rbsp, &pps->bottom_right[i]);
			}
			break;
		case 3: case 4: case 5:
			rbsp_bit(rbsp, &pps->slice_group_change_direction_flag);
			rbsp_uev(rbsp, &pps->slice_group_change_rate_minus1);
			break;
		case 6:
			rbsp_uev(rbsp, &pps->pic_size_in_map_units_minus1);
			for (i = 0; i < pps->pic_size_in_map_units_minus1; i++)
				rbsp_bits(rbsp,
					  order_base_2(pps->num_slice_groups_minus1 + 1),
					  &pps->slice_group_id[i]);
			break;
		default:
			break;
		}
	}
	rbsp_uev(rbsp, &pps->num_ref_idx_l0_default_active_minus1);
	rbsp_uev(rbsp, &pps->num_ref_idx_l1_default_active_minus1);
	rbsp_bit(rbsp, &pps->weighted_pred_flag);
	rbsp_bits(rbsp, 2, &pps->weighted_bipred_idc);
	rbsp_sev(rbsp, &pps->pic_init_qp_minus26);
	rbsp_sev(rbsp, &pps->pic_init_qs_minus26);
	rbsp_sev(rbsp, &pps->chroma_qp_index_offset);
	rbsp_bit(rbsp, &pps->deblocking_filter_control_present_flag);
	rbsp_bit(rbsp, &pps->constrained_intra_pred_flag);
	rbsp_bit(rbsp, &pps->redundant_pic_cnt_present_flag);
	if (/* more_rbsp_data() */ false) {
		rbsp_bit(rbsp, &pps->transform_8x8_mode_flag);
		rbsp_bit(rbsp, &pps->pic_scaling_matrix_present_flag);
		if (pps->pic_scaling_matrix_present_flag)
			rbsp->error = -EINVAL;
		rbsp_sev(rbsp, &pps->second_chroma_qp_index_offset);
	}
}

static void nal_h264_rbsp_slice_hdr(struct rbsp *rbsp,
			struct nal_h264_slice_hdr *s, struct nal_h264_sps spss[],
			struct nal_h264_pps ppss[])
{
	int i, n, sz;
	struct nal_h264_sps *sps;
	struct nal_h264_pps *pps;

	rbsp_bits(rbsp, 1, NULL);
	rbsp_bits(rbsp, 2, &s->nal_ref_idc);
	rbsp_bits(rbsp, 5, &s->nal_unit_type);
	rbsp_uev(rbsp, &s->first_mb_in_slice);
	rbsp_uev(rbsp, &s->slice_type);
	rbsp_uev(rbsp, &s->pic_parameter_set_id);

	s->slice_type %= 5;

	if (s->pic_parameter_set_id >= 256) {
		rbsp->error = -EINVAL;
		return;
	}

	pps = &ppss[s->pic_parameter_set_id];
	sps = &spss[pps->seq_parameter_set_id];

	if (/* !more_rbsp_data() */ false)
		return;

	sz = sps->log2_max_frame_num_minus4 + 4;
	rbsp_bits(rbsp, sz, &s->frame_num);

	if (!sps->frame_mbs_only_flag) {
		rbsp_bit(rbsp, &s->field_pic_flag);
		if (s->field_pic_flag) {
			rbsp_bit(rbsp, &s->bottom_field_flag);
			/* interlaced mb mode not supported */
			return;
		}
	}
	if (s->nal_unit_type == AL_NUT_AVC_VCL_IDR)
		rbsp_uev(rbsp, &s->idr_pic_id);
	switch (sps->pic_order_cnt_type) {
	case 0:
		sz = sps->log2_max_pic_order_cnt_lsb_minus4 + 4;
		rbsp_bits(rbsp, sz, &s->pic_order_cnt_lsb);
		if (pps->bottom_field_pic_order_in_frame_present_flag &&
			!s->field_pic_flag) {
			rbsp_sev(rbsp, &s->delta_pic_order_cnt_bottom);
		}
		break;
	case 1:
		if (!sps->delta_pic_order_always_zero_flag) {
			rbsp_sev(rbsp, &s->delta_pic_order_cnt[0]);
			if (pps->bottom_field_pic_order_in_frame_present_flag &&
				!s->field_pic_flag)
				rbsp_sev(rbsp, &s->delta_pic_order_cnt[1]);
		}
		break;
	}
	if (pps->redundant_pic_cnt_present_flag)
		rbsp_uev(rbsp, &s->redundant_pic_cnt);
	if (s->slice_type == V4L2_H264_SLICE_TYPE_B)
		rbsp_bit(rbsp, &s->direct_spatial_mv_pred_flag);
	if (s->slice_type != V4L2_H264_SLICE_TYPE_I) {
		rbsp_bit(rbsp, &s->num_ref_idx_active_override_flag);
		if (s->num_ref_idx_active_override_flag) {
			rbsp_uev(rbsp, &s->num_ref_idx_l0_active_minus1);
			if (s->slice_type == V4L2_H264_SLICE_TYPE_B)
				rbsp_uev(rbsp, &s->num_ref_idx_l1_active_minus1);
		} else {
			s->num_ref_idx_l0_active_minus1 =
						pps->num_ref_idx_l0_default_active_minus1;
			s->num_ref_idx_l1_active_minus1 =
						pps->num_ref_idx_l1_default_active_minus1;
		}
	}

	if (/* !more_rbsp_data() */ false)
		return;

	/* parse reference pic list reodering syntax */
	if (s->slice_type != V4L2_H264_SLICE_TYPE_I) {
		unsigned int *abs_diff_pic_num_minus1_l0 = s->abs_diff_pic_num_minus1_l0;
		unsigned int *long_term_pic_num_l0 = s->long_term_pic_num_l0;

		rbsp_bit(rbsp, &s->ref_pic_list_reordering_flag_l0);
		if (s->ref_pic_list_reordering_flag_l0) {
			for (i = 0; i < AL_MAX_REFERENCE_PICTURE_REORDER; i++) {
				rbsp_uev(rbsp, &s->reordering_of_pic_nums_idc_l0[i]);
				if (s->reordering_of_pic_nums_idc_l0[i] == 0 ||
					s->reordering_of_pic_nums_idc_l0[i] == 1) {
					rbsp_uev(rbsp, abs_diff_pic_num_minus1_l0);
					abs_diff_pic_num_minus1_l0++;
				} else if (s->reordering_of_pic_nums_idc_l0[i] == 2) {
					rbsp_uev(rbsp, long_term_pic_num_l0);
					long_term_pic_num_l0++;
				}
				if (s->reordering_of_pic_nums_idc_l0[i] != 3)
					return;
			}
		}
	}

	if (s->slice_type == V4L2_H264_SLICE_TYPE_B) {
		unsigned int *abs_diff_pic_num_minus1_l1 = s->abs_diff_pic_num_minus1_l1;
		unsigned int *long_term_pic_num_l1 = s->long_term_pic_num_l1;

		rbsp_bit(rbsp, &s->ref_pic_list_reordering_flag_l1);
		if (s->ref_pic_list_reordering_flag_l1) {
			for (i = 0; i < AL_MAX_REFERENCE_PICTURE_REORDER; i++) {
				rbsp_uev(rbsp, &s->reordering_of_pic_nums_idc_l1[i]);
				if (s->reordering_of_pic_nums_idc_l1[i] == 0 ||
					s->reordering_of_pic_nums_idc_l1[i] == 1) {
					rbsp_uev(rbsp, abs_diff_pic_num_minus1_l1);
					abs_diff_pic_num_minus1_l1++;
				} else if (s->reordering_of_pic_nums_idc_l1[i] == 2) {
					rbsp_uev(rbsp, long_term_pic_num_l1);
					long_term_pic_num_l1++;
				}
				if (s->reordering_of_pic_nums_idc_l1[i] != 3)
					return;
			}
		}
	}

	if (/* !more_rbsp_data() */ false)
		return;

	if ((pps->weighted_pred_flag && s->slice_type == V4L2_H264_SLICE_TYPE_P) ||
		(pps->weighted_bipred_idc == 1 && s->slice_type == V4L2_H264_SLICE_TYPE_B)) {
		rbsp_uev(rbsp, &s->wp_table.luma_log2_weight_denom);

		if (sps->chroma_format_idc != 0)
			rbsp_uev(rbsp, &s->wp_table.chroma_log2_weight_denom);

		sz = s->num_ref_idx_l0_active_minus1;
		for (n = 0; n < 2; n++) {
			for (i = 0; i <= sz; i++) {
				rbsp_bit(rbsp, &s->wp_table.wp_coeff[n].luma_weight_flag[i]);
				if (s->wp_table.wp_coeff[n].luma_weight_flag[i]) {
					rbsp_sev(rbsp, &s->wp_table.wp_coeff[n].luma_delta_weight[i]);
					rbsp_sev(rbsp, &s->wp_table.wp_coeff[n].luma_offset[i]);
				}
				if (sps->chroma_format_idc != 0) {
					rbsp_bit(rbsp, &s->wp_table.wp_coeff[n].chroma_weight_flag[i]);
					if (s->wp_table.wp_coeff[n].chroma_weight_flag[i]) {
						rbsp_sev(rbsp,
							&s->wp_table.wp_coeff[n].chroma_delta_weight[i][0]);
						rbsp_sev(rbsp,
							&s->wp_table.wp_coeff[n].chroma_offset[i][0]);
						rbsp_sev(rbsp,
							&s->wp_table.wp_coeff[n].chroma_delta_weight[i][1]);
						rbsp_sev(rbsp,
							&s->wp_table.wp_coeff[n].chroma_offset[i][1]);
					}
				}
			}
			sz = s->num_ref_idx_l1_active_minus1;
			if (s->slice_type != V4L2_H264_SLICE_TYPE_B)
				break;
		}
	}

	if (/* !more_rbsp_data() */ false)
		return;

	if (s->nal_ref_idc != 0) {
		if (s->nal_unit_type == AL_NUT_AVC_VCL_IDR) {
			rbsp_bit(rbsp, &s->no_output_of_prior_pics_flag);
			rbsp_bit(rbsp, &s->long_term_reference_flag);
		} else {
			rbsp_bit(rbsp, &s->adaptive_ref_pic_marking_mode_flag);
			if (s->adaptive_ref_pic_marking_mode_flag) {
				unsigned int *p0 = s->memory_management_control_operation;
				unsigned int *p1 = s->difference_of_pic_nums_minus1;
				unsigned int *p2 = s->long_term_pic_num;
				unsigned int *p3 = s->long_term_frame_idx;
				unsigned int *p4 = s->max_long_term_frame_idx_plus1;

				rbsp_uev(rbsp, p0);
				while (*p0 != 0) {
					switch (*p0) {
					case 1:
						rbsp_uev(rbsp, p1);
						p1++;
						break;
					case 2:
						rbsp_uev(rbsp, p2);
						p2++;
						break;
					case 3:
						rbsp_uev(rbsp, p1);
						rbsp_uev(rbsp, p3);
						p1++;
						p3++;
						break;
					case 4:
						rbsp_uev(rbsp, p4);
						p4++;
						break;
					case 6:
						rbsp_uev(rbsp, p3);
						p3++;
						break;
					}
					rbsp_uev(rbsp, p0);
				}
			}
		}
	}

	if (pps->entropy_coding_mode_flag && s->slice_type != V4L2_H264_SLICE_TYPE_I)
		rbsp_uev(rbsp, &s->cabac_init_idc);
	rbsp_sev(rbsp, &s->slice_qp_delta);
	if (pps->deblocking_filter_control_present_flag) {
		rbsp_uev(rbsp, &s->disable_deblocking_filter_idc);
		if (s->disable_deblocking_filter_idc != 1) {
			rbsp_sev(rbsp, &s->slice_alpha_c0_offset_div2);
			rbsp_sev(rbsp, &s->slice_beta_offset_div2);
		}
	}
}

/**
 * nal_h264_write_sps() - Write SPS NAL unit into RBSP format
 * @dev: device pointer
 * @dest: the buffer that is filled with RBSP data
 * @n: maximum size of @dest in bytes
 * @sps: &struct nal_h264_sps to convert to RBSP
 *
 * Convert @sps to RBSP data and write it into @dest.
 *
 * The size of the SPS NAL unit is not known in advance and this function will
 * fail, if @dest does not hold sufficient space for the SPS NAL unit.
 *
 * Return: number of bytes written to @dest or negative error code
 */
ssize_t nal_h264_write_sps(const struct device *dev,
			   void *dest, size_t n, struct nal_h264_sps *sps)
{
	struct rbsp rbsp;
	unsigned int forbidden_zero_bit = 0;
	unsigned int nal_ref_idc = 0;
	unsigned int nal_unit_type = AL_NUT_AVC_SPS;

	if (!dest)
		return -EINVAL;

	rbsp_init(&rbsp, dest, n, &write);

	nal_h264_write_start_code_prefix(&rbsp);

	rbsp_bit(&rbsp, &forbidden_zero_bit);
	rbsp_bits(&rbsp, 2, &nal_ref_idc);
	rbsp_bits(&rbsp, 5, &nal_unit_type);

	nal_h264_rbsp_sps(&rbsp, sps);

	rbsp_trailing_bits(&rbsp);

	if (rbsp.error)
		return rbsp.error;

	return DIV_ROUND_UP(rbsp.pos, 8);
}
EXPORT_SYMBOL_GPL(nal_h264_write_sps);

/**
 * nal_h264_read_sps() - Read SPS NAL unit from RBSP format
 * @dev: device pointer
 * @sps: the &struct nal_h264_sps to fill from the RBSP data
 * @src: the buffer that contains the RBSP data
 * @n: size of @src in bytes
 *
 * Read RBSP data from @src and use it to fill @sps.
 *
 * Return: number of bytes read from @src or negative error code
 */
ssize_t nal_h264_read_sps(const struct device *dev,
			  struct nal_h264_sps *sps, void *src, size_t n)
{
	struct rbsp rbsp;
	unsigned int forbidden_zero_bit;
	unsigned int nal_ref_idc;
	unsigned int nal_unit_type;

	if (!src)
		return -EINVAL;

	rbsp_init(&rbsp, src, n, &read);

	nal_h264_read_start_code_prefix(&rbsp);

	rbsp_bit(&rbsp, &forbidden_zero_bit);
	rbsp_bits(&rbsp, 2, &nal_ref_idc);
	rbsp_bits(&rbsp, 5, &nal_unit_type);

	if (rbsp.error ||
	    forbidden_zero_bit != 0 ||
	    nal_ref_idc != 0 ||
	    nal_unit_type != AL_NUT_AVC_SPS)
		return -EINVAL;

	nal_h264_rbsp_sps(&rbsp, sps);

	rbsp_trailing_bits(&rbsp);

	if (rbsp.error)
		return rbsp.error;

	return DIV_ROUND_UP(rbsp.pos, 8);
}
EXPORT_SYMBOL_GPL(nal_h264_read_sps);

/**
 * nal_h264_write_pps() - Write PPS NAL unit into RBSP format
 * @dev: device pointer
 * @dest: the buffer that is filled with RBSP data
 * @n: maximum size of @dest in bytes
 * @pps: &struct nal_h264_pps to convert to RBSP
 *
 * Convert @pps to RBSP data and write it into @dest.
 *
 * The size of the PPS NAL unit is not known in advance and this function will
 * fail, if @dest does not hold sufficient space for the PPS NAL unit.
 *
 * Return: number of bytes written to @dest or negative error code
 */
ssize_t nal_h264_write_pps(const struct device *dev,
			   void *dest, size_t n, struct nal_h264_pps *pps)
{
	struct rbsp rbsp;
	unsigned int forbidden_zero_bit = 0;
	unsigned int nal_ref_idc = 0;
	unsigned int nal_unit_type = AL_NUT_AVC_PPS;

	if (!dest)
		return -EINVAL;

	rbsp_init(&rbsp, dest, n, &write);

	nal_h264_write_start_code_prefix(&rbsp);

	/* NAL unit header */
	rbsp_bit(&rbsp, &forbidden_zero_bit);
	rbsp_bits(&rbsp, 2, &nal_ref_idc);
	rbsp_bits(&rbsp, 5, &nal_unit_type);

	nal_h264_rbsp_pps(&rbsp, pps);

	rbsp_trailing_bits(&rbsp);

	if (rbsp.error)
		return rbsp.error;

	return DIV_ROUND_UP(rbsp.pos, 8);
}
EXPORT_SYMBOL_GPL(nal_h264_write_pps);

/**
 * nal_h264_read_pps() - Read PPS NAL unit from RBSP format
 * @dev: device pointer
 * @pps: the &struct nal_h264_pps to fill from the RBSP data
 * @src: the buffer that contains the RBSP data
 * @n: size of @src in bytes
 *
 * Read RBSP data from @src and use it to fill @pps.
 *
 * Return: number of bytes read from @src or negative error code
 */
ssize_t nal_h264_read_pps(const struct device *dev,
			  struct nal_h264_pps *pps, void *src, size_t n)
{
	struct rbsp rbsp;

	if (!src)
		return -EINVAL;

	rbsp_init(&rbsp, src, n, &read);

	nal_h264_read_start_code_prefix(&rbsp);

	/* NAL unit header */
	rbsp.pos += 8;

	nal_h264_rbsp_pps(&rbsp, pps);

	rbsp_trailing_bits(&rbsp);

	if (rbsp.error)
		return rbsp.error;

	return DIV_ROUND_UP(rbsp.pos, 8);
}
EXPORT_SYMBOL_GPL(nal_h264_read_pps);

/**
 * nal_h264_read_slice_hdr() - Read slice header from RBSP format
 * @dev: device pointer
 * @slice_hdr: the &struct nal_h264_slice_hdr to fill from the RBSP data
 * @src: the buffer that contains the RBSP data
 * @n: size of @src in bytes
 *
 * Read RBSP data from @src and use it to fill @slice_hdr.
 *
 * Return: number of bytes read from @src or negative error code
 */
ssize_t nal_h264_read_slice_hdr(const struct device *dev,
			  struct nal_h264_slice_hdr *slice_hdr, struct nal_h264_sps sps[],
			  struct nal_h264_pps pps[], void *src, size_t n)
{
	struct rbsp rbsp;

	if (!src)
		return -EINVAL;

	rbsp_init(&rbsp, src, n, &read);

	nal_h264_read_start_code_prefix(&rbsp);

	/* NAL unit header */
	//rbsp.pos += 8;

	nal_h264_rbsp_slice_hdr(&rbsp, slice_hdr, sps, pps);

	rbsp_trailing_bits(&rbsp);

	if (rbsp.error)
		return rbsp.error;

	return DIV_ROUND_UP(rbsp.pos, 8);
}
EXPORT_SYMBOL_GPL(nal_h264_read_slice_hdr);

/**
 * nal_h264_write_filler() - Write filler data RBSP
 * @dev: device pointer
 * @dest: buffer to fill with filler data
 * @n: size of the buffer to fill with filler data
 *
 * Write a filler data RBSP to @dest with a size of @n bytes and return the
 * number of written filler data bytes.
 *
 * Use this function to generate dummy data in an RBSP data stream that can be
 * safely ignored by h264 decoders.
 *
 * The RBSP format of the filler data is specified in Rec. ITU-T H.264
 * (04/2017) 7.3.2.7 Filler data RBSP syntax.
 *
 * Return: number of filler data bytes (including marker) or negative error
 */
ssize_t nal_h264_write_filler(const struct device *dev, void *dest, size_t n)
{
	struct rbsp rbsp;
	unsigned int forbidden_zero_bit = 0;
	unsigned int nal_ref_idc = 0;
	unsigned int nal_unit_type = AL_NUT_AVC_FD;

	if (!dest)
		return -EINVAL;

	rbsp_init(&rbsp, dest, n, &write);

	nal_h264_write_start_code_prefix(&rbsp);

	rbsp_bit(&rbsp, &forbidden_zero_bit);
	rbsp_bits(&rbsp, 2, &nal_ref_idc);
	rbsp_bits(&rbsp, 5, &nal_unit_type);

	nal_h264_write_filler_data(&rbsp);

	rbsp_trailing_bits(&rbsp);

	return DIV_ROUND_UP(rbsp.pos, 8);
}
EXPORT_SYMBOL_GPL(nal_h264_write_filler);

/**
 * nal_h264_read_filler() - Read filler data RBSP
 * @dev: device pointer
 * @src: buffer with RBSP data that is read
 * @n: maximum size of src that shall be read
 *
 * Read a filler data RBSP from @src up to a maximum size of @n bytes and
 * return the size of the filler data in bytes including the marker.
 *
 * This function is used to parse filler data and skip the respective bytes in
 * the RBSP data.
 *
 * The RBSP format of the filler data is specified in Rec. ITU-T H.264
 * (04/2017) 7.3.2.7 Filler data RBSP syntax.
 *
 * Return: number of filler data bytes (including marker) or negative error
 */
ssize_t nal_h264_read_filler(const struct device *dev, void *src, size_t n)
{
	struct rbsp rbsp;
	unsigned int forbidden_zero_bit;
	unsigned int nal_ref_idc;
	unsigned int nal_unit_type;

	if (!src)
		return -EINVAL;

	rbsp_init(&rbsp, src, n, &read);

	nal_h264_read_start_code_prefix(&rbsp);

	rbsp_bit(&rbsp, &forbidden_zero_bit);
	rbsp_bits(&rbsp, 2, &nal_ref_idc);
	rbsp_bits(&rbsp, 5, &nal_unit_type);

	if (rbsp.error)
		return rbsp.error;
	if (forbidden_zero_bit != 0 ||
	    nal_ref_idc != 0 ||
	    nal_unit_type != AL_NUT_AVC_FD)
		return -EINVAL;

	nal_h264_read_filler_data(&rbsp);
	rbsp_trailing_bits(&rbsp);

	if (rbsp.error)
		return rbsp.error;

	return DIV_ROUND_UP(rbsp.pos, 8);
}
EXPORT_SYMBOL_GPL(nal_h264_read_filler);
