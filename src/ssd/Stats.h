#ifndef STATS_H
#define STATS_H

#include "SSD_Defs.h"

namespace SSD_Components
{
	class Stats
	{
	public:
		static void Init_stats(unsigned int channel_no, unsigned int chip_no_per_channel, unsigned int die_no_per_chip, unsigned int plane_no_per_die, unsigned int block_no_per_plane, unsigned int page_no_per_block, unsigned int max_allowed_block_erase_count);
		static void Clear_stats(unsigned int channel_no, unsigned int chip_no_per_channel, unsigned int die_no_per_chip, unsigned int plane_no_per_die, unsigned int block_no_per_plane, unsigned int page_no_per_block, unsigned int max_allowed_block_erase_count);
		static unsigned long IssuedReadCMD, IssuedCopybackReadCMD, IssuedInterleaveReadCMD, IssuedMultiplaneReadCMD, IssuedMultiplaneCopybackReadCMD;
		static unsigned long IssuedProgramCMD, IssuedInterleaveProgramCMD, IssuedMultiplaneProgramCMD, IssuedInterleaveMultiplaneProgramCMD, IssuedCopybackProgramCMD, IssuedMultiplaneCopybackProgramCMD;
		static unsigned long IssuedEraseCMD, IssuedInterleaveEraseCMD, IssuedMultiplaneEraseCMD, IssuedInterleaveMultiplaneEraseCMD;

		static unsigned long IssuedSuspendProgramCMD, IssuedSuspendEraseCMD;

		static unsigned long Total_flash_reads_for_mapping, Total_flash_writes_for_mapping;
		static unsigned long Total_flash_reads_for_mapping_per_stream[MAX_SUPPORT_STREAMS], Total_flash_writes_for_mapping_per_stream[MAX_SUPPORT_STREAMS];
		static unsigned long Total_flash_repeat_reads_for_mapping;

		static unsigned int CMT_hits, readTR_CMT_hits, writeTR_CMT_hits;
		static unsigned int CMT_miss, readTR_CMT_miss, writeTR_CMT_miss;
		static unsigned int total_CMT_queries, total_readTR_CMT_queries, total_writeTR_CMT_queries;
		
		static unsigned int CMT_hits_per_stream[MAX_SUPPORT_STREAMS], readTR_CMT_hits_per_stream[MAX_SUPPORT_STREAMS], writeTR_CMT_hits_per_stream[MAX_SUPPORT_STREAMS];
		static unsigned int CMT_miss_per_stream[MAX_SUPPORT_STREAMS], readTR_CMT_miss_per_stream[MAX_SUPPORT_STREAMS], writeTR_CMT_miss_per_stream[MAX_SUPPORT_STREAMS];
		static unsigned int total_CMT_queries_per_stream[MAX_SUPPORT_STREAMS], total_readTR_CMT_queries_per_stream[MAX_SUPPORT_STREAMS], total_writeTR_CMT_queries_per_stream[MAX_SUPPORT_STREAMS];
		

		static unsigned int Total_gc_executions, Total_gc_executions_per_stream[MAX_SUPPORT_STREAMS];
		static unsigned int Total_page_movements_for_gc, Total_gc_page_movements_per_stream[MAX_SUPPORT_STREAMS];

		static unsigned int Total_wl_executions, Total_wl_executions_per_stream[MAX_SUPPORT_STREAMS];
		static unsigned int Total_page_movements_for_wl, Total_wl_page_movements_per_stream[MAX_SUPPORT_STREAMS];

		static unsigned long SLC_user_reads, TLC_user_reads;
		static unsigned long SLC_user_writes, TLC_user_writes, TLC_bypass_writes;
		static unsigned long SLC_physical_reads, SLC_physical_writes, SLC_physical_erases;
		static unsigned long TLC_physical_reads, TLC_physical_writes, TLC_physical_erases;
		static unsigned long SLC_gc_executions, SLC_gc_page_movements, SLC_gc_erases;
		static unsigned long SLC_gc_background_pages, SLC_gc_urgent_pages;
		static unsigned long SLC_bypass_activations;
		static sim_time_type SLC_bypass_total_time;
		static unsigned long TLC_gc_completions;
		static unsigned long Post_TLC_GC_user_io, Post_TLC_GC_tlc_gc, Post_TLC_GC_slc_gc, Post_TLC_GC_other;

		static unsigned int***** Block_erase_histogram;
	};
}

#endif // !STATS_H
