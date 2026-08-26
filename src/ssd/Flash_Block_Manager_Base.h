#ifndef BLOCK_POOL_MANAGER_BASE_H
#define BLOCK_POOL_MANAGER_BASE_H

#include <list>
#include <cstdint>
#include <map>
#include <queue>
#include <set>
#include <unordered_map>
#include <vector>
#include "../nvm_chip/flash_memory/FlashTypes.h"
#include "../nvm_chip/flash_memory/Physical_Page_Address.h"
#include "GC_and_WL_Unit_Base.h"
#include "../nvm_chip/flash_memory/FlashTypes.h"

namespace SSD_Components
{
#define All_VALID_PAGE 0x0000000000000000ULL
	class GC_and_WL_Unit_Base;
	/*
	* Block_Service_Status is used to impelement a state machine for each physical block in order to
	* eliminate race conditions between GC page movements and normal user I/O requests.
	* Allowed transitions:
	* 1: IDLE -> GC, IDLE -> USER
	* 2: GC -> IDLE, GC -> GC_UWAIT
	* 3: USER -> IDLE, USER -> GC_USER
	* 4: GC_UWAIT -> GC, GC_UWAIT -> GC_UWAIT
	* 5: GC_USER -> GC
	*/
	enum class Block_Service_Status {IDLE, GC_WL, USER, GC_USER, GC_UWAIT, GC_USER_UWAIT};
	
	class Block_Pool_Slot_Type
	{
	public:
		flash_block_ID_type BlockID;
		flash_page_ID_type Current_page_write_index;
		Block_Service_Status Current_status;
		unsigned int Invalid_page_count;
		unsigned int Semi_page_count;
		unsigned int Erase_count;
		static unsigned int Page_vector_size;
		uint64_t* Invalid_page_bitmap;//A bit sequence that keeps track of valid/invalid status of pages in the block. A "0" means valid, and a "1" means invalid.
		uint64_t* Semi_page_bitmap;
		stream_id_type Stream_id = NO_STREAM;
		bool Holds_mapping_data = false;
		bool Holds_RAID_data = false;
		bool Has_ongoing_gc_wl = false;
		NVM_Transaction_Flash_ER* Erase_transaction;
		bool Hot_block = false;//Used for hot/cold separation mentioned in the "On the necessity of hot and cold data identification to reduce the write amplification in flash-based SSDs", Perf. Eval., 2014.
		bool Is_slc = false;
		unsigned int Usable_page_count = 0;
		int Ongoing_user_read_count;
		int Ongoing_user_program_count;
		void Erase();

		bool isWL = false;
	};

	class PlaneBookKeepingType
	{
	public:
		unsigned int Total_pages_count;
		unsigned int Free_pages_count;
		unsigned int Valid_pages_count;
		unsigned int Invalid_pages_count;
		Block_Pool_Slot_Type* Blocks;
		std::multimap<unsigned int, Block_Pool_Slot_Type*> Free_block_pool;
		std::multimap<unsigned int, Block_Pool_Slot_Type*> Free_slc_block_pool;
		Block_Pool_Slot_Type** Data_wf, ** GC_wf; //The write frontier blocks for data and GC pages. MQSim adopts Double Write Frontier approach for user and GC writes which is shown very advantages in: B. Van Houdt, "On the necessity of hot and cold data identification to reduce the write amplification in flash - based SSDs", Perf. Eval., 2014
		Block_Pool_Slot_Type** SLC_wf;
		Block_Pool_Slot_Type** Translation_wf; //The write frontier blocks for translation GC pages
		Block_Pool_Slot_Type* Raid_wf;
		std::queue<flash_block_ID_type> Block_usage_history;//A fifo queue that keeps track of flash blocks based on their usage history
		std::set<flash_block_ID_type> Ongoing_erase_operations;
		Block_Pool_Slot_Type* Get_a_free_block(stream_id_type stream_id, bool for_mapping_data);
		Block_Pool_Slot_Type* Get_a_free_slc_block(stream_id_type stream_id);
		unsigned int Get_free_block_pool_size();
		void Check_bookkeeping_correctness(const NVM::FlashMemory::Physical_Page_Address& plane_address);
		void Add_to_free_block_pool(Block_Pool_Slot_Type* block, bool consider_dynamic_wl);
	};

	class Flash_Block_Manager_Base
	{
		friend class Address_Mapping_Unit_Page_Level;
		friend class Address_Mapping_Unit_Page_Level_And_RAID;
		friend class GC_and_WL_Unit_Page_Level;
		friend class GC_and_WL_Unit_Base;
	public:
		Flash_Block_Manager_Base(GC_and_WL_Unit_Base* gc_and_wl_unit, unsigned int max_allowed_block_erase_count, unsigned int total_concurrent_streams_no,
			unsigned int channel_count, unsigned int chip_no_per_channel, unsigned int die_no_per_chip, unsigned int plane_no_per_die,
			unsigned int block_no_per_plane, unsigned int page_no_per_block, unsigned int tlc_block_no_per_plane = 0);
		virtual ~Flash_Block_Manager_Base();
		virtual void Allocate_block_and_page_in_plane_for_user_write(const stream_id_type streamID, NVM::FlashMemory::Physical_Page_Address& address) = 0;
		virtual void Allocate_block_and_page_in_plane_for_tlc_write(const stream_id_type streamID, NVM::FlashMemory::Physical_Page_Address& address) = 0;
		virtual void Allocate_block_and_page_in_plane_for_gc_write(const stream_id_type streamID, NVM::FlashMemory::Physical_Page_Address& address) = 0;
		virtual void Allocate_block_and_page_in_plane_for_translation_write(const stream_id_type streamID, NVM::FlashMemory::Physical_Page_Address& address, bool is_for_gc) = 0;
		virtual void Allocate_block_and_page_in_plane_for_RAID_write(const stream_id_type streamID, NVM::FlashMemory::Physical_Page_Address& address, bool is_for_gc) = 0;
		virtual void Allocate_Pages_in_block_and_invalidate_remaining_for_preconditioning(const stream_id_type stream_id, const NVM::FlashMemory::Physical_Page_Address& plane_address, std::vector<NVM::FlashMemory::Physical_Page_Address>& page_addresses) = 0;
		virtual void Invalidate_page_in_block(const stream_id_type streamID,
			const NVM::FlashMemory::Physical_Page_Address& address,
			bool is_host_update = false) = 0;
		virtual void Invalidate_page_in_block_for_preconditioning(const stream_id_type streamID, const NVM::FlashMemory::Physical_Page_Address& address) = 0;
		
		virtual void Add_erased_block_to_pool(const NVM::FlashMemory::Physical_Page_Address& address) = 0;
		virtual unsigned int Get_pool_size(const NVM::FlashMemory::Physical_Page_Address& plane_address) = 0;
		flash_block_ID_type Get_coldest_block_id(const NVM::FlashMemory::Physical_Page_Address& plane_address);
		unsigned int Get_min_max_erase_difference(const NVM::FlashMemory::Physical_Page_Address& plane_address);
		void Set_GC_and_WL_Unit(GC_and_WL_Unit_Base* );
		PlaneBookKeepingType* Get_plane_bookkeeping_entry(const NVM::FlashMemory::Physical_Page_Address& plane_address);
		bool Block_has_ongoing_gc_wl(const NVM::FlashMemory::Physical_Page_Address& block_address);//Checks if there is an ongoing gc for block_address
		bool Can_execute_gc_wl(const NVM::FlashMemory::Physical_Page_Address& block_address);//Checks if the gc request can be executed on block_address (there shouldn't be any ongoing user read/program requests targeting block_address)
		void GC_WL_started(const NVM::FlashMemory::Physical_Page_Address& block_address);//Updates the block bookkeeping record
		void GC_WL_finished(const NVM::FlashMemory::Physical_Page_Address& block_address);//Updates the block bookkeeping record
		void Read_transaction_issued(const NVM::FlashMemory::Physical_Page_Address& page_address);//Updates the block bookkeeping record
		void Read_transaction_serviced(const NVM::FlashMemory::Physical_Page_Address& page_address);//Updates the block bookkeeping record
		void Program_transaction_serviced(const NVM::FlashMemory::Physical_Page_Address& page_address);//Updates the block bookkeeping record
		bool Is_having_ongoing_program(const NVM::FlashMemory::Physical_Page_Address& block_address);//Cheks if block has any ongoing program request
		bool Is_page_valid(Block_Pool_Slot_Type* block, flash_page_ID_type page_id);//Make the page invalid in the block bookkeeping record
		bool has_Ongoing_user_program(const NVM::FlashMemory::Physical_Page_Address& block_address);

		virtual void Semilate_page_in_block(const stream_id_type stream_id, const NVM::FlashMemory::Physical_Page_Address& page_address);
		virtual bool Is_page_Semil(Block_Pool_Slot_Type* block, flash_page_ID_type page_id);
		virtual void Insemilate_page_in_block(const stream_id_type stream_id, const NVM::FlashMemory::Physical_Page_Address& page_address);
		bool Plane_has_ongoing_gc_wl(NVM::FlashMemory::Physical_Page_Address block_address);
		bool Is_slc_block(const NVM::FlashMemory::Physical_Page_Address& address) const;
		bool Is_slc_cache_enabled() const { return slc_cache_enabled; }
		bool Has_writable_slc_page(const stream_id_type stream_id) const;
		bool Select_global_tlc_plane(const stream_id_type stream_id, bool for_gc,
			NVM::FlashMemory::Physical_Page_Address& plane_address,
			bool enforce_user_admission = true);
		void Reset_tlc_page_allocation_count();
		void Register_closed_slc_block(const NVM::FlashMemory::Physical_Page_Address& block_address);
		bool Dequeue_oldest_closed_slc_block(NVM::FlashMemory::Physical_Page_Address& block_address);
		bool Dequeue_idle_slc_block(bool occupancy_threshold_reached,
			NVM::FlashMemory::Physical_Page_Address& block_address);
		void Record_closed_slc_block_host_invalidation(
			const NVM::FlashMemory::Physical_Page_Address& block_address);
		void Enter_slc_bypass();
		bool Is_slc_bypass_active() const { return slc_bypass_active; }
		unsigned int Get_tlc_block_count() const { return tlc_block_no_per_plane; }
		unsigned int Get_slc_block_count() const { return block_no_per_plane - tlc_block_no_per_plane; }
		unsigned int Get_slc_pages_per_block() const { return slc_pages_no_per_block; }
		bool Is_slc_free_space_below_percent(double threshold_percent) const;
		unsigned long long Get_total_slc_pages() const { return total_slc_pages; }
		unsigned long long Get_free_slc_pages() const { return free_slc_pages; }
		unsigned long long Get_total_free_tlc_blocks() const;
		unsigned long long Get_total_free_slc_blocks() const;
		sim_time_type Get_slc_bypass_time() const;
	protected:
		PlaneBookKeepingType ****plane_manager;//Keeps track of plane block usage information
		GC_and_WL_Unit_Base *gc_and_wl_unit;
		unsigned int max_allowed_block_erase_count;
		unsigned int total_concurrent_streams_no;
		unsigned int channel_count;
		unsigned int chip_no_per_channel;
		unsigned int die_no_per_chip;
		unsigned int plane_no_per_die;
		unsigned int block_no_per_plane;
		unsigned int tlc_block_no_per_plane;
		unsigned int pages_no_per_block;
		unsigned int slc_pages_no_per_block;
		bool slc_cache_enabled;
		bool slc_bypass_active;
		sim_time_type slc_bypass_start_time;
		unsigned long long total_slc_pages;
		unsigned long long free_slc_pages;
			// Linear index of the parallel unit that should receive the next SLC
			// write. The index order is channel, chip, die, plane so consecutive
		// writes spread across channels and chips before reusing a unit.
		unsigned long long slc_next_allocation_unit;
		// TLC user and GC writes share this accounting so placement balances the
		// total number of physical TLC page programs across all parallel units.
		unsigned long long tlc_next_allocation_unit;
		std::vector<unsigned long long> tlc_page_allocation_count;
		using SlcBlockAddressList =
			std::list<NVM::FlashMemory::Physical_Page_Address>;
		enum class SlcTemperaturePosition { FRONT, OVERLAP, MIDDLE, BACK };
		enum class SlcGcCandidateClass { HOT, WARM, COLD };
		struct ClosedSlcBlockTrackingEntry
		{
			SlcBlockAddressList::iterator fifo_position;
			SlcBlockAddressList::iterator lru_position;
			SlcTemperaturePosition fifo_temperature_position;
			SlcTemperaturePosition lru_temperature_position;
			SlcGcCandidateClass candidate_class;
			unsigned long long fifo_sequence;
		};
		// Each ordering is kept in contiguous temperature partitions.  The
		// overlap partition preserves the original rounded 50% boundary when the
		// maximum SLC-block count is odd.  This
		// avoids rescanning all closed SLC blocks when selecting a GC victim.
		SlcBlockAddressList closed_slc_block_fifo_front;
		SlcBlockAddressList closed_slc_block_fifo_overlap;
		SlcBlockAddressList closed_slc_block_fifo_middle;
		SlcBlockAddressList closed_slc_block_fifo_back;
		SlcBlockAddressList closed_slc_block_lru_front;
		SlcBlockAddressList closed_slc_block_lru_overlap;
		SlcBlockAddressList closed_slc_block_lru_middle;
		SlcBlockAddressList closed_slc_block_lru_back;
		std::unordered_map<unsigned long long, ClosedSlcBlockTrackingEntry>
			closed_slc_block_tracking;
		// Maps are ordered by FIFO close sequence, so begin() is the oldest
		// candidate of that class without another FIFO traversal.
		std::map<unsigned long long, unsigned long long>
			closed_slc_warm_candidates;
		std::map<unsigned long long, unsigned long long>
			closed_slc_cold_candidates;
		unsigned long long next_closed_slc_block_fifo_sequence = 0;
		bool Plane_has_writable_tlc_page(const PlaneBookKeepingType& plane_record,
			const stream_id_type stream_id, bool for_gc) const;
		unsigned long long Get_slc_block_tracking_key(
			const NVM::FlashMemory::Physical_Page_Address& block_address) const;
		size_t Get_slc_temperature_region_size() const;
		size_t Get_slc_front_only_region_size() const;
		size_t Get_slc_overlap_region_size() const;
		size_t Get_slc_middle_region_size() const;
		void Rebalance_slc_ordering(SlcBlockAddressList& front,
			SlcBlockAddressList& overlap, SlcBlockAddressList& middle,
			SlcBlockAddressList& back, bool is_fifo_ordering);
		void Set_slc_temperature_position(unsigned long long tracking_key,
			SlcTemperaturePosition position, bool is_fifo_ordering);
		void Refresh_slc_gc_candidate(unsigned long long tracking_key);
		SlcBlockAddressList& Get_slc_ordering_list(
			SlcTemperaturePosition position, bool is_fifo_ordering);
		const SlcBlockAddressList& Get_slc_ordering_list(
			SlcTemperaturePosition position, bool is_fifo_ordering) const;
		bool Dequeue_tracked_closed_slc_block(unsigned long long tracking_key,
			NVM::FlashMemory::Physical_Page_Address& block_address);
		unsigned long long Get_plane_linear_index(
			const NVM::FlashMemory::Physical_Page_Address& plane_address) const;
		void Record_tlc_page_allocation(
			const NVM::FlashMemory::Physical_Page_Address& page_address);
		void program_transaction_issued(const NVM::FlashMemory::Physical_Page_Address& page_address);//Updates the block bookkeeping record
	};
}

#endif//!BLOCK_POOL_MANAGER_BASE_H
