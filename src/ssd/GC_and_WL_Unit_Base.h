#ifndef GC_AND_WL_UNIT_BASE_H
#define GC_AND_WL_UNIT_BASE_H

#include "../sim/Sim_Object.h"
#include "../nvm_chip/flash_memory/Flash_Chip.h"
#include "../nvm_chip/flash_memory/Physical_Page_Address.h"
#include "Address_Mapping_Unit_Base.h"
#include "Flash_Block_Manager_Base.h"
#include "TSU_Base.h"
#include "NVM_PHY_ONFI.h"

#include <deque>
#include <fstream>
#include <stdlib.h>
#include <string>


namespace SSD_Components
{
	enum class GC_Block_Selection_Policy_Type {
		GREEDY,
		RGA,						/*The randomized-greedy algorithm described in: "B. Van Houdt, A Mean Field Model
									for a Class of Garbage Collection Algorithms in Flash - based Solid State Drives,
									SIGMETRICS, 2013" and "Stochastic Modeling of Large-Scale Solid-State Storage
									Systems: Analysis, Design Tradeoffs and Optimization, SIGMETRICS, 2013".*/
		RANDOM, RANDOM_P, RANDOM_PP,/*The RANDOM, RANDOM+, and RANDOM++ algorithms described in: "B. Van Houdt, A Mean
									Field Model  for a Class of Garbage Collection Algorithms in Flash - based Solid
									State Drives, SIGMETRICS, 2013".*/
		FIFO						/*The FIFO algortihm described in P. Desnoyers, "Analytic  Modeling  of  SSD Write
									Performance, SYSTOR, 2012".*/
	};

	class Address_Mapping_Unit_Base;
	class Flash_Block_Manager_Base;
	class TSU_Base;
	class NVM_PHY_ONFI;
	class PlaneBookKeepingType;
	class Block_Pool_Slot_Type;

	/*
	* This class implements thet the Garbage Collection and Wear Leveling module of MQSim.
	*/
	class GC_and_WL_Unit_Base : public MQSimEngine::Sim_Object
	{
	public:
		GC_and_WL_Unit_Base(const sim_object_id_type& id, 
			Address_Mapping_Unit_Base* address_mapping_unit, Flash_Block_Manager_Base* block_manager, TSU_Base* tsu, NVM_PHY_ONFI* flash_controller,
			GC_Block_Selection_Policy_Type block_selection_policy, double gc_threshold,	bool preemptible_gc_enabled, double gc_hard_threshold,
			unsigned int channel_count, unsigned int chip_no_per_channel, unsigned int die_no_per_chip, unsigned int plane_no_per_die,
			unsigned int block_no_per_plane, unsigned int page_no_per_block, unsigned int sector_no_per_page,
			bool use_copyback, double rho, unsigned int max_ongoing_gc_reqs_per_plane,
			bool dynamic_wearleveling_enabled, bool static_wearleveling_enabled, unsigned int static_wearleveling_threshold, int seed);
		
		virtual ~GC_and_WL_Unit_Base(){
			printf("%f\n", static_cast<double>(timeAll) / static_cast<double>(desGC));
			// std::cout << "GC_and_WL_Unit_Base " <<  << std::endl;
			std::cout << "GC_and_WL_Unit_Base " << timeAll << std::endl;
			std::cout << "GC_and_WL_Unit_Base " << desGC << std::endl;
		}

		void Setup_triggers();
		void Start_simulation();
		void Validate_simulation_config();
		void Execute_simulator_event(MQSimEngine::Sim_Event*);

		virtual bool GC_is_in_urgent_mode(const NVM::FlashMemory::Flash_Chip*) = 0;
		virtual void Check_gc_required(const unsigned int BlockPoolSize, const NVM::FlashMemory::Physical_Page_Address& planeAddress) = 0;
		void Check_slc_gc_required();
		void Try_to_schedule_background_slc_gc();
		void Record_slc_user_write();
		void Reset_slc_dynamic_statistics();
		void Set_slc_threshold_history_output(const std::string& output_file_path);
		double Get_slc_dynamic_short_rate_pages_per_second() const;
		double Get_slc_dynamic_long_rate_pages_per_second() const;
		double Get_slc_dynamic_predicted_rate_pages_per_second() const;
		double Get_slc_dynamic_short_occupancy_threshold() const;
		double Get_slc_dynamic_occupancy_threshold() const;
		double Get_slc_dynamic_min_observed_threshold() const;
		double Get_slc_dynamic_max_observed_threshold() const;
		double Get_slc_history_compensation_factor() const;
		unsigned long long Get_slc_dynamic_reserve_pages() const;
		unsigned long long Get_slc_dynamic_fixed_window_updates() const;
		unsigned long long Get_slc_debt_history_window_count() const;
		long long Get_slc_dynamic_current_pressure_pages() const;
		unsigned long long Get_slc_dynamic_current_debt_pages() const;
		unsigned long long Get_slc_dynamic_current_consecutive_debt_windows() const;
		unsigned long long Get_slc_history_longest_continuous_debt_windows() const;
		unsigned long long Get_slc_history_running_net_debt_pages() const;
		unsigned long long Get_slc_long_history_max_net_debt_pages() const;
		unsigned long long Get_slc_dynamic_short_reserve_pages() const;
		unsigned long long Get_slc_dynamic_reclaimed_pages() const;
		GC_Block_Selection_Policy_Type Get_gc_policy();
		unsigned int Get_GC_policy_specific_parameter();//Returns the parameter specific to the GC block selection policy: threshold for random_pp, set_size for RGA
		unsigned int Get_minimum_number_of_free_pages_before_GC();
		bool Use_dynamic_wearleveling();
		bool Use_static_wearleveling();
		bool Stop_servicing_tlc_writes(const NVM::FlashMemory::Physical_Page_Address& plane_address);
		bool is_safe_gc_wl_candidate(const PlaneBookKeepingType* pbke, const flash_block_ID_type gc_wl_candidate_block_id);//Checks if block_address is a safe candidate for gc execution, i.e., 1) it is not a write frontier, and 2) there is no ongoing program operation
	protected:
		GC_Block_Selection_Policy_Type block_selection_policy;
		static GC_and_WL_Unit_Base * _my_instance;
		NVM_PHY_ONFI* flash_controller;
		Address_Mapping_Unit_Base* address_mapping_unit;
		Flash_Block_Manager_Base* block_manager;
		TSU_Base* tsu;
		
		bool force_gc;
		bool slc_gc_active;
		bool slc_gc_background_page_in_flight;
		bool slc_gc_urgent_requested;
		bool slc_gc_background_erase_submitted;
		bool slc_gc_background_promoted;
		flash_page_ID_type slc_gc_next_page_id;
		NVM::FlashMemory::Physical_Page_Address slc_gc_victim_address;
		NVM_Transaction_Flash_ER* slc_gc_background_erase_transaction;
		struct SlcDebtWindowRecord {
			unsigned long long slc_host_writes;
			unsigned long long reclaimed_slc_pages;
			unsigned long long reserve_pages;
			unsigned long long free_slc_pages;
			long long pressure_pages;
			unsigned long long running_net_debt_pages;
		};
		unsigned long long slc_user_write_count;
		unsigned long long slc_reclaimed_page_count;
		unsigned long long slc_rate_window_start_write_count;
		unsigned long long slc_rate_window_start_reclaimed_page_count;
		sim_time_type slc_rate_window_start_time;
		std::deque<unsigned long long> slc_short_rate_history;
		std::deque<SlcDebtWindowRecord> slc_debt_history;
		std::ofstream slc_threshold_history_output;
		unsigned long long slc_long_write_sum;
		sim_time_type slc_long_time_sum;
		double slc_short_rate;
		double slc_long_rate;
		double slc_predicted_rate;
		double slc_dynamic_short_occupancy_threshold;
		double slc_dynamic_occupancy_threshold;
		double slc_dynamic_min_observed_threshold;
		double slc_dynamic_max_observed_threshold;
		double slc_dynamic_free_threshold_percent;
		double slc_history_compensation_factor;
		unsigned long long slc_dynamic_short_reserve_pages;
		unsigned long long slc_dynamic_reserve_pages;
		unsigned long long slc_dynamic_fixed_window_updates;
		long long slc_dynamic_current_pressure_pages;
		unsigned long long slc_dynamic_current_debt_pages;
		unsigned long long slc_dynamic_current_consecutive_debt_windows;
		unsigned long long slc_history_running_net_debt_pages;
		unsigned long long slc_history_longest_continuous_debt_windows;
		unsigned long long slc_long_history_max_net_debt_pages;
		double gc_threshold;//As the ratio of free pages to the total number of physical pages
		unsigned int block_pool_gc_threshold;
		static void handle_transaction_serviced_signal_from_PHY(NVM_Transaction_Flash* transaction);
		static void gc_continue(NVM_Transaction_Flash* transaction, PlaneBookKeepingType* pbke);
		static void create_gc_wl_transaction(NVM::FlashMemory::Physical_Page_Address gc_wl_candidate_address, PlaneBookKeepingType* pbke,
			NVM_Transaction_Flash_ER* existing_erase_transaction = NULL);
		void promote_background_slc_gc_to_urgent();
		bool schedule_one_background_slc_gc_page();
		void submit_background_slc_gc_erase();
		void reset_background_slc_gc_state();
		void record_slc_reclaimed_pages(unsigned long long reclaimed_pages);
		void advance_slc_fixed_rate_windows(sim_time_type now);
		void write_slc_threshold_history_record(unsigned long long writes,
			unsigned long long reclaimed_pages,
			const SlcDebtWindowRecord& current_window);
		void update_slc_dynamic_threshold();
		void update_slc_history_compensation();
		void update_slc_current_debt_summary();
		bool check_static_wl_required(const NVM::FlashMemory::Physical_Page_Address plane_address);
		void run_static_wearleveling(const NVM::FlashMemory::Physical_Page_Address plane_address);
		bool Stop_servicing_tlc_writes_inter(const NVM::FlashMemory::Physical_Page_Address& plane_address);
		bool use_copyback;
		bool dynamic_wearleveling_enabled;
		bool static_wearleveling_enabled;
		unsigned int static_wearleveling_threshold;

		//Used to implement: "Preemptible I/O Scheduling of Garbage Collection for Solid State Drives", TCAD 2013.
		bool preemptible_gc_enabled;
		double gc_hard_threshold;
		unsigned int block_pool_gc_hard_threshold;
		unsigned int max_ongoing_gc_reqs_per_plane;//This value has two important usages: 1) maximum number of concurrent gc operations per plane, and 2) the value that determines urgent GC execution when there is a shortage of flash blocks. If the block bool size drops below this value, all incomming user writes should be blocked

		//Following variabels are used based on the type of GC block selection policy
		unsigned int rga_set_size;//The number of random flash blocks that are radnomly selected 
		Utils::RandomGenerator random_generator;
		std::queue<Block_Pool_Slot_Type*> block_usage_fifo;
		unsigned int random_pp_threshold;

		unsigned int channel_count;
		unsigned int chip_no_per_channel;
		unsigned int die_no_per_chip;
		unsigned int plane_no_per_die;
		unsigned int block_no_per_plane;
		unsigned int pages_no_per_block;
		unsigned int sector_no_per_page;

		unsigned long long desGC = 0;
		sim_time_type timeAll = 0;
	};
}

#endif // !GC_AND_WL_UNIT_BASE_H
