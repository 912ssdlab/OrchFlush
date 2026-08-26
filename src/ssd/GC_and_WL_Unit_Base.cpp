#include "GC_and_WL_Unit_Base.h"
#include "Address_Mapping_Unit_Page_Level_And_RAID.h"
#include "Data_Cache_Manager_Flash_RAID.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace SSD_Components
{
	namespace
	{
		constexpr double SLC_IDLE_FLUSH_SAFETY_FACTOR = 1;
		constexpr sim_time_type SLC_RATE_SHORT_PERIOD = 60ULL * ONE_SECOND;
		constexpr sim_time_type SLC_IDLE_FLUSH_HORIZON = SLC_RATE_SHORT_PERIOD;
		constexpr sim_time_type SLC_RATE_LONG_PERIOD = 16ULL * 60ULL * ONE_SECOND;
		constexpr unsigned int SLC_RATE_LONG_WINDOW_BUCKETS =
			SLC_RATE_LONG_PERIOD / SLC_RATE_SHORT_PERIOD;
		constexpr sim_time_type SLC_DEBT_HISTORY_PERIOD = 100ULL * 60ULL * ONE_SECOND;
		constexpr unsigned int SLC_DEBT_HISTORY_WINDOW_BUCKETS =
			SLC_DEBT_HISTORY_PERIOD / SLC_RATE_SHORT_PERIOD;
		// Keep collecting the 100-minute history, but temporarily disable its
		// contribution to the effective SLC reserve threshold.
		constexpr double SLC_HISTORY_COMPENSATION_GAIN = 0.5;
		static_assert(SLC_RATE_LONG_PERIOD % SLC_RATE_SHORT_PERIOD == 0,
			"The long SLC rate window must contain complete short windows");
		static_assert(SLC_DEBT_HISTORY_PERIOD % SLC_RATE_SHORT_PERIOD == 0,
			"The SLC debt history must contain complete short windows");
		constexpr double SLC_IDLE_FLUSH_MIN_THRESHOLD = 0.20;
		constexpr double SLC_IDLE_FLUSH_MAX_THRESHOLD = 0.95;
		constexpr double SLC_IDLE_FLUSH_INITIAL_THRESHOLD = 0.20;
	}

	GC_and_WL_Unit_Base* GC_and_WL_Unit_Base::_my_instance;
	
	GC_and_WL_Unit_Base::GC_and_WL_Unit_Base(const sim_object_id_type& id,
		Address_Mapping_Unit_Base* address_mapping_unit, Flash_Block_Manager_Base* block_manager, TSU_Base* tsu, NVM_PHY_ONFI* flash_controller,
		GC_Block_Selection_Policy_Type block_selection_policy, double gc_threshold, bool preemptible_gc_enabled, double gc_hard_threshold,
		unsigned int channel_count, unsigned int chip_no_per_channel, unsigned int die_no_per_chip, unsigned int plane_no_per_die,
		unsigned int block_no_per_plane, unsigned int page_no_per_block, unsigned int sector_no_per_page, 
		bool use_copyback, double rho, unsigned int max_ongoing_gc_reqs_per_plane, bool dynamic_wearleveling_enabled, bool static_wearleveling_enabled, unsigned int static_wearleveling_threshold, int seed) :
		Sim_Object(id), address_mapping_unit(address_mapping_unit), block_manager(block_manager), tsu(tsu), flash_controller(flash_controller), force_gc(false), slc_gc_active(false),
		slc_gc_background_page_in_flight(false), slc_gc_urgent_requested(false),
		slc_gc_background_erase_submitted(false), slc_gc_background_promoted(false),
		slc_gc_next_page_id(0), slc_gc_background_erase_transaction(NULL),
		slc_user_write_count(0), slc_reclaimed_page_count(0),
		slc_rate_window_start_write_count(0),
		slc_rate_window_start_reclaimed_page_count(0),
		slc_rate_window_start_time(0),
		slc_long_write_sum(0), slc_long_time_sum(0), slc_short_rate(0),
		slc_long_rate(0), slc_predicted_rate(0),
		slc_dynamic_short_occupancy_threshold(SLC_IDLE_FLUSH_INITIAL_THRESHOLD),
		slc_dynamic_occupancy_threshold(SLC_IDLE_FLUSH_INITIAL_THRESHOLD),
		slc_dynamic_min_observed_threshold(SLC_IDLE_FLUSH_INITIAL_THRESHOLD),
		slc_dynamic_max_observed_threshold(SLC_IDLE_FLUSH_INITIAL_THRESHOLD),
		slc_dynamic_free_threshold_percent(
			100.0 * (1.0 - SLC_IDLE_FLUSH_INITIAL_THRESHOLD)),
		slc_history_compensation_factor(1.0),
		slc_dynamic_short_reserve_pages(0),
		slc_dynamic_reserve_pages(0),
		slc_dynamic_fixed_window_updates(0),
		slc_dynamic_current_pressure_pages(0), slc_dynamic_current_debt_pages(0),
		slc_dynamic_current_consecutive_debt_windows(0),
		slc_history_running_net_debt_pages(0),
		slc_history_longest_continuous_debt_windows(0),
		slc_long_history_max_net_debt_pages(0),
		block_selection_policy(block_selection_policy), gc_threshold(gc_threshold),	use_copyback(use_copyback), 
		preemptible_gc_enabled(preemptible_gc_enabled), gc_hard_threshold(gc_hard_threshold),
		random_generator(seed), max_ongoing_gc_reqs_per_plane(max_ongoing_gc_reqs_per_plane),
		channel_count(channel_count), chip_no_per_channel(chip_no_per_channel), die_no_per_chip(die_no_per_chip), plane_no_per_die(plane_no_per_die),
		block_no_per_plane(block_no_per_plane), pages_no_per_block(page_no_per_block), sector_no_per_page(sector_no_per_page),
		dynamic_wearleveling_enabled(dynamic_wearleveling_enabled), static_wearleveling_enabled(static_wearleveling_enabled), static_wearleveling_threshold(static_wearleveling_threshold)
	{
		_my_instance = this;
		const unsigned long long total_slc_pages = block_manager->Get_total_slc_pages();
		if (total_slc_pages > 0) {
			slc_dynamic_short_reserve_pages = (unsigned long long)std::ceil(
				(1.0L - SLC_IDLE_FLUSH_INITIAL_THRESHOLD) * total_slc_pages);
			slc_dynamic_reserve_pages = slc_dynamic_short_reserve_pages;
			slc_dynamic_free_threshold_percent = 100.0
				* (double)slc_dynamic_reserve_pages / (double)total_slc_pages;
			slc_dynamic_short_occupancy_threshold = 1.0
				- (double)slc_dynamic_short_reserve_pages / (double)total_slc_pages;
			slc_dynamic_occupancy_threshold = 1.0
				- (double)slc_dynamic_reserve_pages / (double)total_slc_pages;
		}
		block_pool_gc_threshold = (unsigned int)(gc_threshold * (double)block_no_per_plane);
		if (block_pool_gc_threshold < 1) {
			block_pool_gc_threshold = 1;
		}
		block_pool_gc_hard_threshold = (unsigned int)(gc_hard_threshold * (double)block_no_per_plane);
		if (block_pool_gc_hard_threshold < 1) {
			block_pool_gc_hard_threshold = 1;
		}
		random_pp_threshold = (unsigned int)(rho * pages_no_per_block);
		if (block_pool_gc_threshold < max_ongoing_gc_reqs_per_plane) {
			block_pool_gc_threshold = max_ongoing_gc_reqs_per_plane;
		}
	}

	void GC_and_WL_Unit_Base::Record_slc_user_write()
	{
		advance_slc_fixed_rate_windows(Simulator->Time());
		slc_user_write_count++;
	}

	void GC_and_WL_Unit_Base::Reset_slc_dynamic_statistics()
	{
		slc_user_write_count = 0;
		slc_reclaimed_page_count = 0;
		slc_rate_window_start_write_count = 0;
		slc_rate_window_start_reclaimed_page_count = 0;
		slc_rate_window_start_time = Simulator->Time();
		slc_short_rate_history.clear();
		slc_debt_history.clear();
		slc_long_write_sum = 0;
		slc_long_time_sum = 0;
		slc_short_rate = 0;
		slc_long_rate = 0;
		slc_predicted_rate = 0;
		slc_history_compensation_factor = 1.0;
		slc_dynamic_fixed_window_updates = 0;
		slc_dynamic_current_pressure_pages = 0;
		slc_dynamic_current_debt_pages = 0;
		slc_dynamic_current_consecutive_debt_windows = 0;
		slc_history_running_net_debt_pages = 0;
		slc_history_longest_continuous_debt_windows = 0;
		slc_long_history_max_net_debt_pages = 0;

		const unsigned long long total_slc_pages =
			block_manager->Get_total_slc_pages();
		if (total_slc_pages == 0) {
			slc_dynamic_short_reserve_pages = 0;
			slc_dynamic_reserve_pages = 0;
			slc_dynamic_short_occupancy_threshold =
				SLC_IDLE_FLUSH_INITIAL_THRESHOLD;
			slc_dynamic_occupancy_threshold =
				SLC_IDLE_FLUSH_INITIAL_THRESHOLD;
			slc_dynamic_free_threshold_percent = 100.0
				* (1.0 - SLC_IDLE_FLUSH_INITIAL_THRESHOLD);
		} else {
			slc_dynamic_short_reserve_pages =
				(unsigned long long)std::ceil(
					(1.0L - SLC_IDLE_FLUSH_INITIAL_THRESHOLD)
					* total_slc_pages);
			slc_dynamic_reserve_pages = slc_dynamic_short_reserve_pages;
			slc_dynamic_free_threshold_percent = 100.0
				* (double)slc_dynamic_reserve_pages / (double)total_slc_pages;
			slc_dynamic_short_occupancy_threshold = 1.0
				- (double)slc_dynamic_short_reserve_pages
					/ (double)total_slc_pages;
			slc_dynamic_occupancy_threshold =
				slc_dynamic_short_occupancy_threshold;
		}
		slc_dynamic_min_observed_threshold =
			slc_dynamic_occupancy_threshold;
		slc_dynamic_max_observed_threshold =
			slc_dynamic_occupancy_threshold;
	}

	void GC_and_WL_Unit_Base::record_slc_reclaimed_pages(
		unsigned long long reclaimed_pages)
	{
		advance_slc_fixed_rate_windows(Simulator->Time());
		slc_reclaimed_page_count += reclaimed_pages;
	}

	void GC_and_WL_Unit_Base::Set_slc_threshold_history_output(
		const std::string& output_file_path)
	{
		if (slc_threshold_history_output.is_open())
			slc_threshold_history_output.close();
		slc_threshold_history_output.clear();
		slc_threshold_history_output.open(output_file_path.c_str(),
			std::ios::out | std::ios::trunc);
		if (!slc_threshold_history_output)
			throw std::runtime_error(
				"Cannot open SLC threshold history file: " + output_file_path);

		slc_threshold_history_output
			<< "window_index,window_end_time_ns,window_end_time_minutes,"
			<< "slc_host_write_pages,reclaimed_slc_pages,"
			<< "rate_1min_pages_per_second,rate_10min_pages_per_second,"
			<< "predicted_rate_pages_per_second,running_net_debt_pages,"
			<< "history_window_count,longest_continuous_debt_windows,"
			<< "history_compensation_factor,base_reserve_pages,"
			<< "final_reserve_pages,base_occupancy_threshold,"
			<< "final_occupancy_threshold,free_slc_pages,pressure_pages\n";
		slc_threshold_history_output << std::setprecision(12);
		slc_threshold_history_output.flush();
	}

	void GC_and_WL_Unit_Base::write_slc_threshold_history_record(
		unsigned long long writes, unsigned long long reclaimed_pages,
		const SlcDebtWindowRecord& current_window)
	{
		if (!slc_threshold_history_output.is_open())
			return;

		slc_threshold_history_output
			<< slc_dynamic_fixed_window_updates << ','
			<< slc_rate_window_start_time << ','
			<< (long double)slc_rate_window_start_time
				/ (60.0L * (long double)ONE_SECOND) << ','
			<< writes << ','
			<< reclaimed_pages << ','
			<< Get_slc_dynamic_short_rate_pages_per_second() << ','
			<< Get_slc_dynamic_long_rate_pages_per_second() << ','
			<< Get_slc_dynamic_predicted_rate_pages_per_second() << ','
			<< slc_history_running_net_debt_pages << ','
			<< slc_debt_history.size() << ','
			<< slc_history_longest_continuous_debt_windows << ','
			<< slc_history_compensation_factor << ','
			<< slc_dynamic_short_reserve_pages << ','
			<< slc_dynamic_reserve_pages << ','
			<< slc_dynamic_short_occupancy_threshold << ','
			<< slc_dynamic_occupancy_threshold << ','
			<< current_window.free_slc_pages << ','
			<< current_window.pressure_pages << '\n';
		slc_threshold_history_output.flush();
		if (!slc_threshold_history_output)
			throw std::runtime_error("Failed to write SLC threshold history");
	}

	void GC_and_WL_Unit_Base::advance_slc_fixed_rate_windows(sim_time_type now)
	{
		if (now < slc_rate_window_start_time)
			return;

		// Advance on SLC writes or idle checks instead of installing a permanent
		// periodic event that could keep MQSim's event queue alive indefinitely.
		while (now - slc_rate_window_start_time >= SLC_RATE_SHORT_PERIOD) {
			const unsigned long long writes =
				slc_user_write_count - slc_rate_window_start_write_count;
			const unsigned long long reclaimed_pages = slc_reclaimed_page_count
				- slc_rate_window_start_reclaimed_page_count;
			slc_short_rate = (double)writes / (double)SLC_RATE_SHORT_PERIOD;
			slc_short_rate_history.push_back(writes);
			slc_long_write_sum += writes;
			while (slc_short_rate_history.size() > SLC_RATE_LONG_WINDOW_BUCKETS) {
				slc_long_write_sum -= slc_short_rate_history.front();
				slc_short_rate_history.pop_front();
			}
			slc_long_time_sum = (sim_time_type)slc_short_rate_history.size()
				* SLC_RATE_SHORT_PERIOD;
			slc_long_rate = slc_long_time_sum == 0 ? 0.0
				: (double)slc_long_write_sum / (double)slc_long_time_sum;

			slc_rate_window_start_time += SLC_RATE_SHORT_PERIOD;
			slc_rate_window_start_write_count = slc_user_write_count;
			slc_rate_window_start_reclaimed_page_count = slc_reclaimed_page_count;
			slc_dynamic_fixed_window_updates++;
			if (writes >= reclaimed_pages) {
				const unsigned long long new_debt = writes - reclaimed_pages;
				if (new_debt > ULLONG_MAX - slc_history_running_net_debt_pages)
					slc_history_running_net_debt_pages = ULLONG_MAX;
				else
					slc_history_running_net_debt_pages += new_debt;
			} else {
				const unsigned long long repaid_debt = reclaimed_pages - writes;
				slc_history_running_net_debt_pages =
					repaid_debt >= slc_history_running_net_debt_pages
						? 0
						: slc_history_running_net_debt_pages - repaid_debt;
			}
			slc_debt_history.push_back({writes, reclaimed_pages,
				0, 0, 0, slc_history_running_net_debt_pages});
			while (slc_debt_history.size() > SLC_DEBT_HISTORY_WINDOW_BUCKETS)
				slc_debt_history.pop_front();
			update_slc_history_compensation();
			update_slc_dynamic_threshold();

			SlcDebtWindowRecord& current_window = slc_debt_history.back();
			current_window.reserve_pages = slc_dynamic_reserve_pages;
			current_window.free_slc_pages = block_manager->Get_free_slc_pages();
			current_window.pressure_pages =
				slc_dynamic_reserve_pages >= current_window.free_slc_pages
					? (long long)(slc_dynamic_reserve_pages - current_window.free_slc_pages)
					: -(long long)(current_window.free_slc_pages - slc_dynamic_reserve_pages);
			update_slc_current_debt_summary();
			write_slc_threshold_history_record(writes, reclaimed_pages,
				current_window);
		}
	}

	void GC_and_WL_Unit_Base::update_slc_history_compensation()
	{
		slc_history_longest_continuous_debt_windows = 0;
		slc_long_history_max_net_debt_pages = 0;
		unsigned long long continuous_debt_windows = 0;
		for (const SlcDebtWindowRecord& record : slc_debt_history) {
			slc_long_history_max_net_debt_pages = std::max(
				slc_long_history_max_net_debt_pages,
				record.running_net_debt_pages);
			if (record.running_net_debt_pages == 0) {
				continuous_debt_windows = 0;
				continue;
			}
			continuous_debt_windows++;
			slc_history_longest_continuous_debt_windows = std::max(
				slc_history_longest_continuous_debt_windows,
				continuous_debt_windows);
		}
		const double history_ratio = slc_debt_history.empty()
			? 0.0
			: (double)slc_history_longest_continuous_debt_windows
				/ (double)slc_debt_history.size();
		slc_history_compensation_factor = 1.0
			+ SLC_HISTORY_COMPENSATION_GAIN * history_ratio;
	}

	void GC_and_WL_Unit_Base::update_slc_current_debt_summary()
	{
		slc_dynamic_current_pressure_pages = 0;
		slc_dynamic_current_debt_pages = 0;
		slc_dynamic_current_consecutive_debt_windows = 0;
		if (!slc_debt_history.empty()) {
			slc_dynamic_current_pressure_pages =
				slc_debt_history.back().pressure_pages;
			if (slc_dynamic_current_pressure_pages > 0)
				slc_dynamic_current_debt_pages =
					(unsigned long long)slc_dynamic_current_pressure_pages;
			for (auto record = slc_debt_history.rbegin();
				record != slc_debt_history.rend() && record->pressure_pages > 0;
				record++)
				slc_dynamic_current_consecutive_debt_windows++;
		}
	}

	void GC_and_WL_Unit_Base::update_slc_dynamic_threshold()
	{
		const unsigned long long total_slc_pages = block_manager->Get_total_slc_pages();
		if (total_slc_pages == 0)
			return;

		slc_predicted_rate = std::max(slc_short_rate, slc_long_rate);
		const long double raw_short_reserve = SLC_IDLE_FLUSH_SAFETY_FACTOR
			* (long double)slc_predicted_rate * SLC_IDLE_FLUSH_HORIZON;
		const long double raw_compensated_reserve = raw_short_reserve
			* (long double)slc_history_compensation_factor;
		auto calculate_threshold = [total_slc_pages](long double raw_reserve,
			unsigned long long& reserve_pages) {
			const unsigned long long minimum_reserve_pages =
				(unsigned long long)std::ceil(
					(1.0L - SLC_IDLE_FLUSH_MAX_THRESHOLD) * total_slc_pages);
			const unsigned long long maximum_reserve_pages =
				(unsigned long long)std::ceil(
					(1.0L - SLC_IDLE_FLUSH_MIN_THRESHOLD) * total_slc_pages);
			const long double bounded_reserve = std::max(
				(long double)minimum_reserve_pages,
				std::min((long double)maximum_reserve_pages, raw_reserve));
			reserve_pages = (unsigned long long)std::ceil(bounded_reserve);
			return 1.0 - (double)reserve_pages / (double)total_slc_pages;
		};
		slc_dynamic_short_occupancy_threshold = calculate_threshold(
			raw_short_reserve, slc_dynamic_short_reserve_pages);
		slc_dynamic_occupancy_threshold = calculate_threshold(
			raw_compensated_reserve, slc_dynamic_reserve_pages);
		slc_dynamic_free_threshold_percent = 100.0
			* (double)slc_dynamic_reserve_pages / (double)total_slc_pages;
		slc_dynamic_min_observed_threshold = std::min(
			slc_dynamic_min_observed_threshold, slc_dynamic_occupancy_threshold);
		slc_dynamic_max_observed_threshold = std::max(
			slc_dynamic_max_observed_threshold, slc_dynamic_occupancy_threshold);
	}

	double GC_and_WL_Unit_Base::Get_slc_dynamic_short_rate_pages_per_second() const
	{
		return slc_short_rate * 1000000000.0;
	}

	double GC_and_WL_Unit_Base::Get_slc_dynamic_long_rate_pages_per_second() const
	{
		return slc_long_rate * 1000000000.0;
	}

	double GC_and_WL_Unit_Base::Get_slc_dynamic_predicted_rate_pages_per_second() const
	{
		return slc_predicted_rate * 1000000000.0;
	}

	double GC_and_WL_Unit_Base::Get_slc_dynamic_short_occupancy_threshold() const
	{
		return slc_dynamic_short_occupancy_threshold;
	}

	double GC_and_WL_Unit_Base::Get_slc_dynamic_occupancy_threshold() const
	{
		return slc_dynamic_occupancy_threshold;
	}

	double GC_and_WL_Unit_Base::Get_slc_dynamic_min_observed_threshold() const
	{
		return slc_dynamic_min_observed_threshold;
	}

	double GC_and_WL_Unit_Base::Get_slc_dynamic_max_observed_threshold() const
	{
		return slc_dynamic_max_observed_threshold;
	}

	double GC_and_WL_Unit_Base::Get_slc_history_compensation_factor() const
	{
		return slc_history_compensation_factor;
	}

	unsigned long long GC_and_WL_Unit_Base::Get_slc_dynamic_reserve_pages() const
	{
		return slc_dynamic_reserve_pages;
	}

	unsigned long long GC_and_WL_Unit_Base::Get_slc_dynamic_fixed_window_updates() const
	{
		return slc_dynamic_fixed_window_updates;
	}

	unsigned long long GC_and_WL_Unit_Base::Get_slc_debt_history_window_count() const
	{
		return slc_debt_history.size();
	}

	long long GC_and_WL_Unit_Base::Get_slc_dynamic_current_pressure_pages() const
	{
		return slc_dynamic_current_pressure_pages;
	}

	unsigned long long GC_and_WL_Unit_Base::Get_slc_dynamic_current_debt_pages() const
	{
		return slc_dynamic_current_debt_pages;
	}

	unsigned long long GC_and_WL_Unit_Base::Get_slc_dynamic_current_consecutive_debt_windows() const
	{
		return slc_dynamic_current_consecutive_debt_windows;
	}

	unsigned long long GC_and_WL_Unit_Base::Get_slc_history_longest_continuous_debt_windows() const
	{
		return slc_history_longest_continuous_debt_windows;
	}

	unsigned long long GC_and_WL_Unit_Base::Get_slc_history_running_net_debt_pages() const
	{
		return slc_history_running_net_debt_pages;
	}

	unsigned long long GC_and_WL_Unit_Base::Get_slc_long_history_max_net_debt_pages() const
	{
		return slc_long_history_max_net_debt_pages;
	}

	unsigned long long GC_and_WL_Unit_Base::Get_slc_dynamic_short_reserve_pages() const
	{
		return slc_dynamic_short_reserve_pages;
	}

	unsigned long long GC_and_WL_Unit_Base::Get_slc_dynamic_reclaimed_pages() const
	{
		return slc_reclaimed_page_count;
	}

	void GC_and_WL_Unit_Base::Setup_triggers()
	{
		Sim_Object::Setup_triggers();
		flash_controller->ConnectToTransactionServicedSignal(handle_transaction_serviced_signal_from_PHY);
	}

	void GC_and_WL_Unit_Base::create_gc_wl_transaction(NVM::FlashMemory::Physical_Page_Address gc_wl_candidate_address, PlaneBookKeepingType* pbke,
		NVM_Transaction_Flash_ER* existing_erase_transaction){
		Block_Pool_Slot_Type* block = &pbke->Blocks[gc_wl_candidate_address.BlockID];
		Transaction_Source_Type gc_source = block->Is_slc ? Transaction_Source_Type::SLC_GC : Transaction_Source_Type::GC_WL;
		
		//_my_instance->block_manager->GC_WL_finished(gc_wl_candidate_address);
		_my_instance->tsu->Prepare_for_transaction_submit();
		NVM_Transaction_Flash_ER* gc_wl_erase_tr = existing_erase_transaction != NULL
			? existing_erase_transaction
			: new NVM_Transaction_Flash_ER(gc_source, block->Stream_id, gc_wl_candidate_address);
		gc_wl_erase_tr->Tier = block->Is_slc ? Flash_Data_Tier::SLC : Flash_Data_Tier::TLC;
		std::vector<std::pair<stream_id_type, LPA_type>> cacheCommits;
		if (1 || block->Current_page_write_index - block->Invalid_page_count > 0) {
			//address_mapping_unit->Lock_physical_block_for_gc(gc_candidate_address);//Lock the block, so no user request can intervene while the GC is progressing
			NVM_Transaction_Flash_RD* gc_wl_read = NULL;
			NVM_Transaction_Flash_WR* gc_wl_write = NULL;
			int Pcount = 0;
			for (flash_page_ID_type pageID = 0; pageID < block->Current_page_write_index; pageID++) {
				gc_wl_candidate_address.PageID = pageID;
				LPA_type lpa = _my_instance->flash_controller->Get_metadata(gc_wl_candidate_address.ChannelID, gc_wl_candidate_address.ChipID, gc_wl_candidate_address.DieID, gc_wl_candidate_address.PlaneID, gc_wl_candidate_address.BlockID, gc_wl_candidate_address.PageID);
				
				if (_my_instance->block_manager->Is_page_valid(block, pageID)) {
					Pcount++;
					if (block->Is_slc)
						Stats::SLC_gc_urgent_pages++;
					if(block->isWL){
						Stats::Total_page_movements_for_wl++;
					}else{
						
					}
					if(block->Holds_RAID_data){
						Address_Mapping_Unit_Page_Level_And_RAID *address_mapping_unit = dynamic_cast<Address_Mapping_Unit_Page_Level_And_RAID*>(_my_instance->address_mapping_unit);
						if(_my_instance->use_copyback){
							gc_wl_write = new NVM_Transaction_Flash_WR(gc_source, address_mapping_unit->get_RAID_stream(gc_wl_candidate_address.ChannelID), _my_instance->sector_no_per_page * SECTOR_SIZE_IN_BYTE,
								NO_LPA, _my_instance->address_mapping_unit->Convert_address_to_ppa(gc_wl_candidate_address), NULL, 0, NULL, 0, INVALID_TIME_STAMP);
							gc_wl_write->ExecutionMode = WriteExecutionModeType::COPYBACK;
							_my_instance->tsu->Submit_transaction(gc_wl_write);
						}else{
							gc_wl_read = new NVM_Transaction_Flash_RD(gc_source, address_mapping_unit->get_RAID_stream(gc_wl_candidate_address.ChannelID), _my_instance->sector_no_per_page * SECTOR_SIZE_IN_BYTE,
								NO_LPA, _my_instance->address_mapping_unit->Convert_address_to_ppa(gc_wl_candidate_address), gc_wl_candidate_address, NULL, 0, NULL, 0, INVALID_TIME_STAMP);
							gc_wl_write = new NVM_Transaction_Flash_WR(gc_source, address_mapping_unit->get_RAID_stream(gc_wl_candidate_address.ChannelID), _my_instance->sector_no_per_page * SECTOR_SIZE_IN_BYTE,
								NO_LPA, NO_PPA, gc_wl_candidate_address, NULL, 0, gc_wl_read, 0, INVALID_TIME_STAMP);
							gc_wl_write->ExecutionMode = WriteExecutionModeType::SIMPLE;
							gc_wl_write->RelatedErase = gc_wl_erase_tr;
							gc_wl_read->RelatedWrite = gc_wl_write;
							
							_my_instance->tsu->Submit_transaction(gc_wl_read);//Only the read transaction would be submitted. The Write transaction is submitted when the read transaction is finished and the LPA of the target page is determined
						}
					}else if (_my_instance->use_copyback) {
						gc_wl_write = new NVM_Transaction_Flash_WR(gc_source, block->Stream_id, _my_instance->sector_no_per_page * SECTOR_SIZE_IN_BYTE,
							NO_LPA, _my_instance->address_mapping_unit->Convert_address_to_ppa(gc_wl_candidate_address), NULL, 0, NULL, 0, INVALID_TIME_STAMP);
						gc_wl_write->ExecutionMode = WriteExecutionModeType::COPYBACK;
						_my_instance->tsu->Submit_transaction(gc_wl_write);
					} else {
						
						gc_wl_read = new NVM_Transaction_Flash_RD(gc_source, block->Stream_id, _my_instance->sector_no_per_page * SECTOR_SIZE_IN_BYTE,
							NO_LPA, _my_instance->address_mapping_unit->Convert_address_to_ppa(gc_wl_candidate_address), gc_wl_candidate_address, NULL, 0, NULL, 0, INVALID_TIME_STAMP);
						gc_wl_write = new NVM_Transaction_Flash_WR(gc_source, block->Stream_id, _my_instance->sector_no_per_page * SECTOR_SIZE_IN_BYTE,
							NO_LPA, NO_PPA, gc_wl_candidate_address, NULL, 0, gc_wl_read, 0, INVALID_TIME_STAMP);
						gc_wl_write->ExecutionMode = WriteExecutionModeType::SIMPLE;
						gc_wl_write->RelatedErase = gc_wl_erase_tr;
						gc_wl_read->RelatedWrite = gc_wl_write;
						_my_instance->tsu->Submit_transaction(gc_wl_read);//Only the read transaction would be submitted. The Write transaction is submitted when the read transaction is finished and the LPA of the target page is determined
					}
					gc_wl_erase_tr->Page_movement_activities.push_back(gc_wl_write);
					if (gc_wl_read != NULL)
						gc_wl_read->Tier = block->Is_slc ? Flash_Data_Tier::SLC : Flash_Data_Tier::TLC;
					gc_wl_write->Tier = Flash_Data_Tier::TLC;
				}else if(_my_instance->block_manager->Is_page_Semil(block, pageID)){
					// std::cout << block->Semi_page_bitmap[pageID / 64] << std::endl;
					Address_Mapping_Unit_Page_Level_And_RAID *address_mapping_unit = dynamic_cast<Address_Mapping_Unit_Page_Level_And_RAID*>(_my_instance->address_mapping_unit);
					LPA_type raidLPA = address_mapping_unit->Get_And_Create_RAIDID(block->Stream_id, lpa);
					stream_id_type stream = (raidLPA & (NO_LPA << 56)) >> 56;
					raidLPA = UNIQUE_KEY_TO_LPN(stream, raidLPA);
					NVM::FlashMemory::Physical_Page_Address addr;
					cacheCommits.emplace_back(stream, raidLPA);
					// address_mapping_unit->Semilate_page(stream, raidLPA); 
				}
			}
			if(Pcount != block->Usable_page_count - block->Invalid_page_count){
				std::cout << "block->Usable_page_count - block->Invalid_page_count " <<std::endl;
			}
		}
		if(cacheCommits.size() > 0){
			// std::cout << "block->Semi_page_count" << block->Semi_page_count << std::endl;
			Data_Cache_Manager_Flash_RAID *dcm = dynamic_cast<Data_Cache_Manager_Flash_RAID*>(_my_instance->address_mapping_unit->get_FTL()->Data_cache_manager);
			dcm->gc_eviction(cacheCommits, gc_wl_erase_tr);
		}
		block->Erase_transaction = gc_wl_erase_tr;
		_my_instance->tsu->Submit_transaction(gc_wl_erase_tr);
		_my_instance->tsu->Schedule();
		
	}

	void GC_and_WL_Unit_Base::Check_slc_gc_required()
	{
		if (slc_gc_active) {
			if (slc_gc_background_erase_transaction != NULL
				&& !slc_gc_background_erase_submitted
				&& !slc_gc_background_promoted) {
				slc_gc_urgent_requested = true;
				if (!slc_gc_background_page_in_flight)
					promote_background_slc_gc_to_urgent();
			}
			return;
		}

		NVM::FlashMemory::Physical_Page_Address victim_address;
		if (!block_manager->Dequeue_oldest_closed_slc_block(victim_address))
			return;

		PlaneBookKeepingType* victim_plane =
			block_manager->Get_plane_bookkeeping_entry(victim_address);
		Block_Pool_Slot_Type* victim = &victim_plane->Blocks[victim_address.BlockID];
		if (!victim->Is_slc
			|| victim->Has_ongoing_gc_wl
			|| victim->Current_page_write_index != victim->Usable_page_count)
			PRINT_ERROR("Invalid block found in the closed SLC FIFO")

		slc_gc_active = true;
		block_manager->GC_WL_started(victim_address);
		victim_plane->Ongoing_erase_operations.insert(victim_address.BlockID);
		address_mapping_unit->Set_barrier_for_accessing_physical_block(victim_address);
		Stats::SLC_gc_executions++;
		if (block_manager->Can_execute_gc_wl(victim_address)) {
			create_gc_wl_transaction(victim_address, victim_plane);
		}
	}

	void GC_and_WL_Unit_Base::Try_to_schedule_background_slc_gc()
	{
		if (!block_manager->Is_slc_cache_enabled())
			return;
		advance_slc_fixed_rate_windows(Simulator->Time());
		// Do not select a new background victim after every workload flow has
		// stopped generating requests and all generated requests are serviced.
		// An already active victim remains eligible for its normal continuation.
		if (Simulator->Is_workload_completed() && !slc_gc_active)
			return;
		if (block_manager->Is_slc_bypass_active()) {
			Check_slc_gc_required();
			return;
		}
		if (slc_gc_active) {
			if (slc_gc_background_erase_transaction == NULL
				|| slc_gc_background_page_in_flight
				|| slc_gc_background_erase_submitted
				|| slc_gc_background_promoted
				|| slc_gc_urgent_requested)
				return;
			schedule_one_background_slc_gc_page();
			return;
		}

		const bool occupancy_threshold_reached =
			block_manager->Is_slc_free_space_below_percent(
				slc_dynamic_free_threshold_percent);
		NVM::FlashMemory::Physical_Page_Address victim_address;
		// Cold blocks (both orderings in the fixed global back region) are always
		// eligible. Warm blocks require the dynamic occupancy threshold, while
		// hot blocks (both orderings in the fixed global front region) are protected.
		// Forced bypass reclaim remains an exhaustion-safety path.
		if (!block_manager->Dequeue_idle_slc_block(
			occupancy_threshold_reached, victim_address))
			return;
		PlaneBookKeepingType* victim_plane =
			block_manager->Get_plane_bookkeeping_entry(victim_address);
		Block_Pool_Slot_Type* victim = &victim_plane->Blocks[victim_address.BlockID];
		if (!victim->Is_slc || victim->Has_ongoing_gc_wl
			|| victim->Current_page_write_index != victim->Usable_page_count)
			PRINT_ERROR("Invalid block found in the closed SLC FIFO")

		slc_gc_active = true;
		slc_gc_victim_address = victim_address;
		slc_gc_next_page_id = 0;
		slc_gc_background_page_in_flight = false;
		slc_gc_urgent_requested = false;
		slc_gc_background_erase_submitted = false;
		slc_gc_background_promoted = false;
		slc_gc_background_erase_transaction = new NVM_Transaction_Flash_ER(
			Transaction_Source_Type::SLC_GC, victim->Stream_id, victim_address);
		slc_gc_background_erase_transaction->Tier = Flash_Data_Tier::SLC;
		Stats::SLC_gc_executions++;
		schedule_one_background_slc_gc_page();
	}

	bool GC_and_WL_Unit_Base::schedule_one_background_slc_gc_page()
	{
		PlaneBookKeepingType* victim_plane =
			block_manager->Get_plane_bookkeeping_entry(slc_gc_victim_address);
		Block_Pool_Slot_Type* victim = &victim_plane->Blocks[slc_gc_victim_address.BlockID];
		while (slc_gc_next_page_id < victim->Current_page_write_index
			&& !block_manager->Is_page_valid(victim, slc_gc_next_page_id))
			slc_gc_next_page_id++;

		if (slc_gc_next_page_id >= victim->Current_page_write_index) {
			submit_background_slc_gc_erase();
			return true;
		}
		if (!block_manager->Can_execute_gc_wl(slc_gc_victim_address))
			return false;

		NVM::FlashMemory::Physical_Page_Address source_address(slc_gc_victim_address);
		source_address.PageID = slc_gc_next_page_id;
		NVM::FlashMemory::Physical_Page_Address target_plane;
		if (!block_manager->Select_global_tlc_plane(victim->Stream_id, true, target_plane, false)
			|| !tsu->Can_start_background_slc_gc(source_address, target_plane))
			return false;

		LPA_type lpa = flash_controller->Get_metadata(source_address.ChannelID,
			source_address.ChipID, source_address.DieID, source_address.PlaneID,
			source_address.BlockID, source_address.PageID);
		PPA_type mapped_ppa;
		page_status_type page_status_bitmap;
		address_mapping_unit->Get_data_mapping_info_for_gc(
			victim->Stream_id, lpa, mapped_ppa, page_status_bitmap);
		if (mapped_ppa != address_mapping_unit->Convert_address_to_ppa(source_address))
			PRINT_ERROR("Inconsistency found when starting background SLC GC page movement!")

		address_mapping_unit->Set_barrier_for_accessing_lpa(victim->Stream_id, lpa);
		NVM_Transaction_Flash_RD* read = new NVM_Transaction_Flash_RD(
			Transaction_Source_Type::SLC_GC, victim->Stream_id,
			sector_no_per_page * SECTOR_SIZE_IN_BYTE, lpa, mapped_ppa,
			source_address, NULL, 0, NULL, 0, INVALID_TIME_STAMP);
		NVM_Transaction_Flash_WR* write = new NVM_Transaction_Flash_WR(
			Transaction_Source_Type::SLC_GC, victim->Stream_id,
			sector_no_per_page * SECTOR_SIZE_IN_BYTE, NO_LPA, NO_PPA,
			source_address, NULL, 0, read, 0, INVALID_TIME_STAMP);
		read->RelatedWrite = write;
		read->Tier = Flash_Data_Tier::SLC;
		write->ExecutionMode = WriteExecutionModeType::SIMPLE;
		write->RelatedErase = slc_gc_background_erase_transaction;
		write->Tier = Flash_Data_Tier::TLC;
		slc_gc_background_erase_transaction->Page_movement_activities.push_back(write);
		slc_gc_next_page_id++;
		slc_gc_background_page_in_flight = true;
		Stats::SLC_gc_background_pages++;

		tsu->Prepare_for_transaction_submit();
		tsu->Submit_transaction(read);
		tsu->Schedule();
		return true;
	}

	void GC_and_WL_Unit_Base::submit_background_slc_gc_erase()
	{
		if (!tsu->Can_start_background_slc_gc(
			slc_gc_victim_address, slc_gc_victim_address)
			|| !block_manager->Can_execute_gc_wl(slc_gc_victim_address))
			return;

		PlaneBookKeepingType* victim_plane =
			block_manager->Get_plane_bookkeeping_entry(slc_gc_victim_address);
		Block_Pool_Slot_Type* victim = &victim_plane->Blocks[slc_gc_victim_address.BlockID];
		block_manager->GC_WL_started(slc_gc_victim_address);
		victim_plane->Ongoing_erase_operations.insert(slc_gc_victim_address.BlockID);
		victim->Erase_transaction = slc_gc_background_erase_transaction;
		slc_gc_background_erase_submitted = true;
		tsu->Prepare_for_transaction_submit();
		tsu->Submit_transaction(slc_gc_background_erase_transaction);
		tsu->Schedule();
	}

	void GC_and_WL_Unit_Base::promote_background_slc_gc_to_urgent()
	{
		if (slc_gc_background_page_in_flight || slc_gc_background_promoted
			|| slc_gc_background_erase_submitted)
			return;
		PlaneBookKeepingType* victim_plane =
			block_manager->Get_plane_bookkeeping_entry(slc_gc_victim_address);
		block_manager->GC_WL_started(slc_gc_victim_address);
		victim_plane->Ongoing_erase_operations.insert(slc_gc_victim_address.BlockID);
		address_mapping_unit->Set_barrier_for_accessing_physical_block(slc_gc_victim_address);
		slc_gc_background_promoted = true;
		if (block_manager->Can_execute_gc_wl(slc_gc_victim_address))
			create_gc_wl_transaction(slc_gc_victim_address, victim_plane,
				slc_gc_background_erase_transaction);
	}

	void GC_and_WL_Unit_Base::reset_background_slc_gc_state()
	{
		slc_gc_background_page_in_flight = false;
		slc_gc_urgent_requested = false;
		slc_gc_background_erase_submitted = false;
		slc_gc_background_promoted = false;
		slc_gc_next_page_id = 0;
		slc_gc_background_erase_transaction = NULL;
	}

	void GC_and_WL_Unit_Base::gc_continue(NVM_Transaction_Flash* transaction, PlaneBookKeepingType* pbke){
		if (_my_instance->block_manager->Block_has_ongoing_gc_wl(transaction->Address)) {
			if (_my_instance->block_manager->Can_execute_gc_wl(transaction->Address)) {
				NVM_Transaction_Flash_ER* existing_erase = NULL;
				if (_my_instance->slc_gc_background_promoted
					&& transaction->Address.ChannelID == _my_instance->slc_gc_victim_address.ChannelID
					&& transaction->Address.ChipID == _my_instance->slc_gc_victim_address.ChipID
					&& transaction->Address.DieID == _my_instance->slc_gc_victim_address.DieID
					&& transaction->Address.PlaneID == _my_instance->slc_gc_victim_address.PlaneID
					&& transaction->Address.BlockID == _my_instance->slc_gc_victim_address.BlockID)
					existing_erase = _my_instance->slc_gc_background_erase_transaction;
				create_gc_wl_transaction(transaction->Address, pbke, existing_erase);
				// std::cout << transaction->Address.ChannelID << " " << transaction->Address.ChipID << " " << transaction->Address.PlaneID << " " << transaction->Address.BlockID << std::endl;
				// if(transaction->Source == Transaction_Source_Type::GC_WL){
				// 	std::cout << "transaction_Source_Type::GC_WL" << std::endl;
				// }
				// throw " 1";
			}
		}
	}
	
	void GC_and_WL_Unit_Base::handle_transaction_serviced_signal_from_PHY(NVM_Transaction_Flash* transaction)
	{
		static int gcCount = 0;
		PlaneBookKeepingType* pbke = &(_my_instance->block_manager->plane_manager[transaction->Address.ChannelID][transaction->Address.ChipID][transaction->Address.DieID][transaction->Address.PlaneID]);
		_my_instance->desGC++;
		_my_instance->timeAll += Simulator->Time() - transaction->Issue_time;
		// if(_my_instance->desGC == 15583283){ // min 15583280 max 15583285    82805107689916 82805107694524
		// 	std::cout << (transaction->Source == Transaction_Source_Type::GC_WL) << std::endl;
		// 	std::cout << transaction << " " << transaction->Stream_id << " " << transaction->LPA << std::endl;
		// 	std::cout << transaction->cacheEj << std::endl;
		// 	std::cout << (transaction->Type == Transaction_Type::READ) << std::endl;
		// 	std::cout << transaction->Address.ChannelID << " " << transaction->Address.ChipID;
		// 	std::cout << " " << transaction->Address.DieID << " " << transaction->Address.PlaneID;
		// 	std::cout << " " << transaction->Address.BlockID << " " << transaction->Address.PageID << std::endl;
		// 	gcCount = 0; // 0x6000028f1e00 0 18446744073709551615 0x600000951ec0 0 538222 538222
		// 	Simulator->Stop_simulation();
		// }
		switch (transaction->Source) {
			case Transaction_Source_Type::USERIO:
			case Transaction_Source_Type::MAPPING:
			case Transaction_Source_Type::CACHE:
				switch (transaction->Type)
				{
					case Transaction_Type::READ:
						_my_instance->block_manager->Read_transaction_serviced(transaction->Address);
						break;
					case Transaction_Type::WRITE:
						_my_instance->block_manager->Program_transaction_serviced(transaction->Address);
						if(_my_instance->address_mapping_unit->Is_has_writes_for_overfull_plane(transaction->Address)){
							_my_instance->address_mapping_unit->Start_servicing_writes_for_overfull_plane(transaction->Address);
							if(_my_instance->address_mapping_unit->Is_has_writes_for_overfull_plane(transaction->Address)
								&& !_my_instance->block_manager->Plane_has_ongoing_gc_wl(transaction->Address))
								_my_instance->Check_gc_required(1 ,transaction->Address);
						}
						break;
					default:
						PRINT_ERROR("Unexpected situation in the GC_and_WL_Unit_Base function!")
				}
				gc_continue(transaction, pbke);
				return;
			default:
				break;
		}
		if(transaction->cacheEj){
			if(--(*transaction->relatedCount) == 0){
				// std::cout << 0 << std::endl;
				// ((NVM_Transaction_Flash_RD*)transaction)->RelatedWrite->RelatedRead = NULL;
				// delete transaction->relatedCount;
			}
			NVM_Transaction_Flash_RD * tr = (NVM_Transaction_Flash_RD*)transaction;
			if(tr->RelatedErase){
				NVM_Transaction_Flash_ER * gc_tr = (NVM_Transaction_Flash_ER*)(tr->RelatedErase);
				gc_tr->Page_movement_activities.remove(tr);
				return;
			}		

			_my_instance->block_manager->Read_transaction_serviced(transaction->Address);
			gc_continue(transaction, pbke);
			
			return;
		}


		switch (transaction->Type) {
			case Transaction_Type::READ:
			{
				PPA_type ppa;
				MPPN_type mppa;
				page_status_type page_status_bitmap;
				if(pbke->Blocks[transaction->Address.BlockID].Holds_RAID_data){
					//_my_instance->address_mapping_unit->Get_RAID_mapping_info_for_gc(transaction->Stream_id, (MVPN_type)transaction->LPA, mppa, page_status_bitmap);
					Address_Mapping_Unit_Page_Level_And_RAID *address_mapping_unit = dynamic_cast<Address_Mapping_Unit_Page_Level_And_RAID*>(_my_instance->address_mapping_unit);
					address_mapping_unit->Get_RAID_mapping_info_for_gc(transaction->Stream_id, transaction->LPA, ppa, page_status_bitmap);
					if (ppa == transaction->PPA){
						_my_instance->tsu->Prepare_for_transaction_submit();
						((NVM_Transaction_Flash_RD*)transaction)->RelatedWrite->write_sectors_bitmap = page_status_bitmap;
						((NVM_Transaction_Flash_RD*)transaction)->RelatedWrite->LPA = transaction->LPA;
						((NVM_Transaction_Flash_RD*)transaction)->RelatedWrite->RelatedRead = NULL;
						
						address_mapping_unit->Allocate_new_page_for_gc(((NVM_Transaction_Flash_RD*)transaction)->RelatedWrite, pbke->Blocks[transaction->Address.BlockID].Holds_mapping_data);
						_my_instance->tsu->Submit_transaction(((NVM_Transaction_Flash_RD*)transaction)->RelatedWrite);
						_my_instance->tsu->Schedule();
					}else{
						std::cout <<transaction->LPA << " " << transaction->PPA << " " << ppa << " " << NO_PPA<<std::endl;
						std::cout << transaction->Stream_id << std::endl;
						std::cout << 111 <<std::endl;
						PRINT_ERROR("Inconsistency found when moving a page for GC/WL!")
					}
				}else if (pbke->Blocks[transaction->Address.BlockID].Holds_mapping_data) {
					
					_my_instance->address_mapping_unit->Get_translation_mapping_info_for_gc(transaction->Stream_id, (MVPN_type)transaction->LPA, mppa, page_status_bitmap);
					//There has been no write on the page since GC start, and it is still valid
					if (mppa == transaction->PPA) {
						_my_instance->tsu->Prepare_for_transaction_submit();
						((NVM_Transaction_Flash_RD*)transaction)->RelatedWrite->write_sectors_bitmap = FULL_PROGRAMMED_PAGE;
						((NVM_Transaction_Flash_RD*)transaction)->RelatedWrite->LPA = transaction->LPA;
						
						((NVM_Transaction_Flash_RD*)transaction)->RelatedWrite->RelatedRead = NULL;
						if (transaction->Source == Transaction_Source_Type::SLC_GC) {
							_my_instance->address_mapping_unit->Allocate_new_page_for_slc_gc(((NVM_Transaction_Flash_RD*)transaction)->RelatedWrite);
							_my_instance->Check_gc_required(
								_my_instance->block_manager->Get_pool_size(((NVM_Transaction_Flash_RD*)transaction)->RelatedWrite->Address),
								((NVM_Transaction_Flash_RD*)transaction)->RelatedWrite->Address);
						} else
							_my_instance->address_mapping_unit->Allocate_new_page_for_gc(((NVM_Transaction_Flash_RD*)transaction)->RelatedWrite, pbke->Blocks[transaction->Address.BlockID].Holds_mapping_data);
						_my_instance->tsu->Submit_transaction(((NVM_Transaction_Flash_RD*)transaction)->RelatedWrite);
						_my_instance->tsu->Schedule();
					} else {
						PRINT_ERROR("Inconsistency found when moving a page for GC/WL!")
					}
				} else {
					_my_instance->address_mapping_unit->Get_data_mapping_info_for_gc(transaction->Stream_id, transaction->LPA, ppa, page_status_bitmap);
					Address_Mapping_Unit_Page_Level_And_RAID *address_mapping_unit = dynamic_cast<Address_Mapping_Unit_Page_Level_And_RAID*>(_my_instance->address_mapping_unit);
					address_mapping_unit = NULL;
					if(address_mapping_unit){
						ppa = address_mapping_unit->Get_ppa(true, transaction->Stream_id, transaction->LPA);
						page_status_bitmap = address_mapping_unit->Get_page_status(true, transaction->Stream_id, transaction->LPA);
						
					}
					//There has been no write on the page since GC start, and it is still valid
					if (ppa == transaction->PPA) {
						_my_instance->tsu->Prepare_for_transaction_submit();
						((NVM_Transaction_Flash_RD*)transaction)->RelatedWrite->write_sectors_bitmap = page_status_bitmap;
						((NVM_Transaction_Flash_RD*)transaction)->RelatedWrite->LPA = transaction->LPA;
						((NVM_Transaction_Flash_RD*)transaction)->RelatedWrite->RelatedRead = NULL;
						// if(address_mapping_unit){
						// 	address_mapping_unit->Allocate_new_page_for_gc(((NVM_Transaction_Flash_RD*)transaction)->RelatedWrite, pbke->Blocks[transaction->Address.BlockID].Holds_mapping_data);
						// }else{
						if (transaction->Source == Transaction_Source_Type::SLC_GC) {
							_my_instance->address_mapping_unit->Allocate_new_page_for_slc_gc(((NVM_Transaction_Flash_RD*)transaction)->RelatedWrite);
							_my_instance->Check_gc_required(
								_my_instance->block_manager->Get_pool_size(((NVM_Transaction_Flash_RD*)transaction)->RelatedWrite->Address),
								((NVM_Transaction_Flash_RD*)transaction)->RelatedWrite->Address);
						} else
							_my_instance->address_mapping_unit->Allocate_new_page_for_gc(((NVM_Transaction_Flash_RD*)transaction)->RelatedWrite, pbke->Blocks[transaction->Address.BlockID].Holds_mapping_data);
						// }
						_my_instance->tsu->Submit_transaction(((NVM_Transaction_Flash_RD*)transaction)->RelatedWrite);
						_my_instance->tsu->Schedule();
					} else {
						std::cout <<transaction->LPA << " " << transaction->PPA << " " << ppa << " " << NO_PPA<<std::endl;
						std::cout << transaction->Stream_id << std::endl;
						PRINT_ERROR("Inconsistency found when moving a page for GC/WL!")
					}

				}
				if(((NVM_Transaction_Flash_RD*)transaction)->RelatedWrite){
					((NVM_Transaction_Flash_RD*)transaction)->RelatedWrite->RelatedRead = NULL;
				}
				break;
			}
			case Transaction_Type::WRITE:
			{
				_my_instance->block_manager->Program_transaction_serviced(transaction->Address);
				gc_continue(transaction, pbke);
				if (_my_instance->address_mapping_unit->Is_has_writes_for_overfull_plane(transaction->Address)
					&& !_my_instance->block_manager->Plane_has_ongoing_gc_wl(transaction->Address)) {
					_my_instance->address_mapping_unit->Start_servicing_writes_for_overfull_plane(transaction->Address);
				}
				if (transaction->Source == Transaction_Source_Type::SLC_GC)
					Stats::SLC_gc_page_movements++;
				else
					Stats::Total_page_movements_for_gc++;
				NVM_Transaction_Flash_ER* related_erase = ((NVM_Transaction_Flash_WR*)transaction)->RelatedErase;
				PlaneBookKeepingType* source_pbke = _my_instance->block_manager->Get_plane_bookkeeping_entry(related_erase->Address);
				Block_Pool_Slot_Type* source_block = &source_pbke->Blocks[related_erase->Address.BlockID];
				if(source_block->Holds_RAID_data){
					auto *address_mapping_unit = dynamic_cast<Address_Mapping_Unit_Page_Level_And_RAID*>(_my_instance->address_mapping_unit);
					stream_id_type streamId = address_mapping_unit->get_RAID_stream(related_erase->Address.ChannelID);
					address_mapping_unit->Remove_barrier_for_accessing_RAID(streamId, transaction->LPA);
				}else if (source_block->Holds_mapping_data) {
					_my_instance->address_mapping_unit->Remove_barrier_for_accessing_mvpn(transaction->Stream_id, (MVPN_type)transaction->LPA);
					DEBUG(Simulator->Time() << ": MVPN=" << (MVPN_type)transaction->LPA << " unlocked!!");
					
				} else {
					_my_instance->address_mapping_unit->Remove_barrier_for_accessing_lpa(transaction->Stream_id, transaction->LPA);
					DEBUG(Simulator->Time() << ": LPA=" << (MVPN_type)transaction->LPA << " unlocked!!");
					//std::cout << Simulator->Time() << ": LPA=" << (MVPN_type)transaction->LPA << " unlocked!!" << std::endl;
				}
				related_erase->Page_movement_activities.remove((NVM_Transaction_Flash_WR*)transaction);
				if (transaction->Source == Transaction_Source_Type::SLC_GC
					&& related_erase == _my_instance->slc_gc_background_erase_transaction
					&& !_my_instance->slc_gc_background_promoted) {
					_my_instance->slc_gc_background_page_in_flight = false;
					if (_my_instance->slc_gc_urgent_requested)
						_my_instance->promote_background_slc_gc_to_urgent();
				}
				break;
			}
			case Transaction_Type::ERASE:
				if (transaction->Source == Transaction_Source_Type::SLC_GC) {
					_my_instance->record_slc_reclaimed_pages(
						pbke->Blocks[transaction->Address.BlockID].Usable_page_count);
					Stats::SLC_gc_erases++;
					_my_instance->slc_gc_active = false;
					if (transaction == _my_instance->slc_gc_background_erase_transaction)
						_my_instance->reset_background_slc_gc_state();
				} else
					_my_instance->tsu->Notify_tlc_gc_completion(transaction->Address);
				pbke->Ongoing_erase_operations.erase(pbke->Ongoing_erase_operations.find(transaction->Address.BlockID));
				_my_instance->block_manager->Add_erased_block_to_pool(transaction->Address);
				_my_instance->block_manager->GC_WL_finished(transaction->Address);
				_my_instance->address_mapping_unit->Start_servicing_writes_for_overfull_plane(transaction->Address);//Must be inovked after above statements since it may lead to flash page consumption for waiting program transactions
				
				// if(++gcCount == 3629){
				// 	gcCount = 0;
				// 	// Simulator->Stop_simulation();
				// }
				if (_my_instance->check_static_wl_required(transaction->Address)) {
					_my_instance->run_static_wearleveling(transaction->Address);
				}
				if (_my_instance->Stop_servicing_tlc_writes_inter(transaction->Address)) {
					_my_instance->Check_gc_required(pbke->Get_free_block_pool_size(), transaction->Address);
				}
				break;
			default:
				break;
		}
		//gc_continue(transaction, pbke);
	}
	

	void GC_and_WL_Unit_Base::Start_simulation()
	{
	}

	void GC_and_WL_Unit_Base::Validate_simulation_config()
	{
	}

	void GC_and_WL_Unit_Base::Execute_simulator_event(MQSimEngine::Sim_Event* ev)
	{
	}

	GC_Block_Selection_Policy_Type GC_and_WL_Unit_Base::Get_gc_policy()
	{
		return block_selection_policy;
	}

	unsigned int GC_and_WL_Unit_Base::Get_GC_policy_specific_parameter()
	{
		switch (block_selection_policy) {
			case GC_Block_Selection_Policy_Type::RGA:
				return rga_set_size;
			case GC_Block_Selection_Policy_Type::RANDOM_PP:
				return random_pp_threshold;
			default:
				break;
		}

		return 0;
	}

	unsigned int GC_and_WL_Unit_Base::Get_minimum_number_of_free_pages_before_GC()
	{
		return block_pool_gc_threshold;
		/*if (preemptible_gc_enabled)
			return block_pool_gc_hard_threshold;
		else return block_pool_gc_threshold;*/
	}

	bool GC_and_WL_Unit_Base::Use_dynamic_wearleveling()
	{
		return dynamic_wearleveling_enabled;
	}

	inline bool GC_and_WL_Unit_Base::Use_static_wearleveling()
	{
		return static_wearleveling_enabled;
	}

	bool GC_and_WL_Unit_Base::Stop_servicing_tlc_writes_inter(const NVM::FlashMemory::Physical_Page_Address& plane_address){
		PlaneBookKeepingType* pbke = &(_my_instance->block_manager->plane_manager[plane_address.ChannelID][plane_address.ChipID][plane_address.DieID][plane_address.PlaneID]);
		if(block_manager->Get_pool_size(plane_address) >= max_ongoing_gc_reqs_per_plane){
			return false;
		}else{
			// TLC admission must not depend on an SLC victim running in this
			// plane. Resource conflicts are resolved later by the TSU.
			for(unsigned int i = 0; i < block_manager->Get_tlc_block_count(); ++i){
				if(pbke->Blocks[i].Has_ongoing_gc_wl){
					return false;
				}
			}
			return true;
		}
	}
	
	bool GC_and_WL_Unit_Base::Stop_servicing_tlc_writes(const NVM::FlashMemory::Physical_Page_Address& plane_address)
	{
		PlaneBookKeepingType* pbke = &(_my_instance->block_manager->plane_manager[plane_address.ChannelID][plane_address.ChipID][plane_address.DieID][plane_address.PlaneID]);
		if(block_manager->Get_pool_size(plane_address) > max_ongoing_gc_reqs_per_plane){
			return false;
		}else{
			// Only TLC state is relevant here. An SLC GC on the same plane may
			// delay execution at the TSU, but it must not reject TLC allocation.
			for(unsigned int i = 0; i < block_manager->Get_tlc_block_count(); ++i){
				if(pbke->Blocks[i].Has_ongoing_gc_wl){
					return true;
				}
				if(pbke->Blocks[i].Ongoing_user_program_count){
					return true;
				}
			}
			// return true;
			// if(block_manager->Get_pool_size(plane_address) <= max_ongoing_gc_reqs_per_plane)
			// 	return true;
				
			return false;
		}
	}

	bool GC_and_WL_Unit_Base::is_safe_gc_wl_candidate(const PlaneBookKeepingType* plane_record, const flash_block_ID_type gc_wl_candidate_block_id)
	{
		//The block shouldn't be a current write frontier
		for (unsigned int stream_id = 0; stream_id < address_mapping_unit->Get_no_of_input_streams(); stream_id++) {
			if ((&plane_record->Blocks[gc_wl_candidate_block_id]) == plane_record->Data_wf[stream_id]
				|| (&plane_record->Blocks[gc_wl_candidate_block_id]) == plane_record->Translation_wf[stream_id]
				|| (&plane_record->Blocks[gc_wl_candidate_block_id]) == plane_record->GC_wf[stream_id]) {
				return false;
			}
		}

		//The block shouldn't have an ongoing program request (all pages must already be written)
		if (plane_record->Blocks[gc_wl_candidate_block_id].Ongoing_user_program_count > 0) {
			return false;
		}

		if (plane_record->Blocks[gc_wl_candidate_block_id].Has_ongoing_gc_wl) {
			return false;
		}

		return true;
	}

	inline bool GC_and_WL_Unit_Base::check_static_wl_required(const NVM::FlashMemory::Physical_Page_Address plane_address)
	{
		return static_wearleveling_enabled && (block_manager->Get_min_max_erase_difference(plane_address) >= static_wearleveling_threshold);
	}

	void GC_and_WL_Unit_Base::run_static_wearleveling(const NVM::FlashMemory::Physical_Page_Address plane_address)
	{
		PlaneBookKeepingType* pbke = block_manager->Get_plane_bookkeeping_entry(plane_address);
		flash_block_ID_type wl_candidate_block_id = block_manager->Get_coldest_block_id(plane_address);
		if (!is_safe_gc_wl_candidate(pbke, wl_candidate_block_id) 
		|| pbke->Ongoing_erase_operations.find(wl_candidate_block_id) != pbke->Ongoing_erase_operations.end()
		|| pbke->Blocks[wl_candidate_block_id].Current_page_write_index != pages_no_per_block) {
			return;
		}

		NVM::FlashMemory::Physical_Page_Address wl_candidate_address(plane_address);
		wl_candidate_address.BlockID = wl_candidate_block_id;
		Block_Pool_Slot_Type* block = &pbke->Blocks[wl_candidate_block_id];

		/*if(wl_candidate_address.ChannelID == 1 && wl_candidate_address.ChipID == 0 && wl_candidate_address.DieID == 0 && wl_candidate_address.PlaneID == 0 && wl_candidate_address.BlockID == 0){
			std::cout << "WL SET " << std::endl;
		}*/
		//Run the state machine to protect against race condition
		block_manager->GC_WL_started(wl_candidate_address);
		
		pbke->Ongoing_erase_operations.insert(wl_candidate_block_id);
		address_mapping_unit->Set_barrier_for_accessing_physical_block(wl_candidate_address);//Lock the block, so no user request can intervene while the GC is progressing
		Stats::Total_wl_executions++;

		pbke->Blocks[wl_candidate_address.BlockID].isWL = true;
		if (block_manager->Can_execute_gc_wl(wl_candidate_address)) {//If there are ongoing requests targeting the candidate block, the gc execution should be postponed
			create_gc_wl_transaction(wl_candidate_address, pbke);
		}/*else{
			std::cout << "WL " << block->Ongoing_user_program_count <<" " << block->Ongoing_user_read_count <<std::endl;
			std::cout << wl_candidate_address.ChannelID << " " << wl_candidate_address.ChipID << " " << wl_candidate_address.DieID << " " << wl_candidate_address.PlaneID << " " << wl_candidate_address.BlockID << std::endl;
		}*/
	}
}
