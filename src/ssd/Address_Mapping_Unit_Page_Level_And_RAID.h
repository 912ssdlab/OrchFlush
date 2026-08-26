#ifndef ADDRESS_MAPPING_UNIT_PAGE_LEVEL_AND_RAID
#define ADDRESS_MAPPING_UNIT_PAGE_LEVEL_AND_RAID


#include "Address_Mapping_Unit_Page_Level.h"

#include <unordered_map>
#include <vector>


namespace SSD_Components
{

class AddressMappingDomain_RAID : public AddressMappingDomain{
public:
    int channelToken = 0;

    std::vector<LPA_type> nowRAIDID;
    AddressMappingDomain_RAID(unsigned int cmt_capacity, unsigned int cmt_entry_size, unsigned int no_of_translation_entries_per_page,
			Cached_Mapping_Table* CMT,
			Flash_Plane_Allocation_Scheme_Type PlaneAllocationScheme,
			flash_channel_ID_type* channel_ids, unsigned int channel_no, flash_chip_ID_type* chip_ids, unsigned int chip_no,
			flash_die_ID_type* die_ids, unsigned int die_no, flash_plane_ID_type* plane_ids, unsigned int plane_no,
			PPA_type total_physical_sectors_no, LHA_type total_logical_sectors_no, unsigned int sectors_no_per_page, int);
};

class Address_Mapping_Unit_Page_Level_And_RAID : public Address_Mapping_Unit_Page_Level{
protected:
    const LPA_type RAID_STREAN = 255;
    const int STRIPE_DATA_NUM = 3;
    const int STRIPE_INFO_LEN = STRIPE_DATA_NUM + 3 + STRIPE_DATA_NUM;

    /*raid*/
    std::vector<int> checkPosRAID;

    std::vector<int> chipToken;
    std::vector<int> dieToken;
    std::vector<int> planeToken;

    std::unordered_map<LPA_type, LPA_type> LPA2RAID;
    std::vector<std::vector<std::vector<LPA_type>>> RAID2LPA; // STRIPE_DATA_NUM is PPA, STRIPE_DATA_NUM + 1 is WrittenStateBitmap, STRIPE_DATA_NUM + 2 is time
    
    /*for dynamic allocate*/
    std::set<LPA_type> Locked_RAIDs;
    std::multimap<LPA_type, NVM_Transaction_Flash*> Read_transactions_behind_RAID_barrier;
	std::multimap<LPA_type, NVM_Transaction_Flash*> Write_transactions_behind_RAID_barrier;

protected:
    void pre_translate_lpa_to_ppa(const stream_id_type stream_id, const LPA_type lpn, const int channel);
    void allocate_plane(stream_id_type Stream_id, LPA_type lpn, NVM::FlashMemory::Physical_Page_Address& targetAddress);
    void allocate_plane_for_user_write(NVM_Transaction_Flash_WR* transaction);
    void allocate_page_in_plane_for_user_write(NVM_Transaction_Flash_WR* transaction, bool is_for_gc);
    
    bool is_lpa_locked_for_gc(stream_id_type stream_id, LPA_type lpa);
    void manage_user_transaction_facing_barrier(NVM_Transaction_Flash* transaction);
    PPA_type online_create_entry_for_reads(LPA_type lpa, const stream_id_type stream_id, NVM::FlashMemory::Physical_Page_Address& read_address, uint64_t read_sectors_bitmap);
    void Update_mapping_info(const bool ideal_mapping, const stream_id_type stream_id, const LPA_type lpa, const PPA_type ppa, const page_status_type page_status_bitmap);
    bool translate_lpa_to_ppa(stream_id_type streamID, NVM_Transaction_Flash* transaction);
    bool query_cmt(NVM_Transaction_Flash* transaction);
    bool Mapping_entry_accessible(const bool ideal_mapping, const stream_id_type stream_id, const LPA_type lpa);
public:
    Address_Mapping_Unit_Page_Level_And_RAID(const sim_object_id_type& id, FTL* ftl, NVM_PHY_ONFI* flash_controller, Flash_Block_Manager_Base* block_manager,
			bool ideal_mapping_table, unsigned int cmt_capacity_in_byte, Flash_Plane_Allocation_Scheme_Type PlaneAllocationScheme,
			unsigned int ConcurrentStreamNo,
			unsigned int ChannelCount, unsigned int chip_no_per_channel, unsigned int DieNoPerChip, unsigned int PlaneNoPerDie,
			std::vector<std::vector<flash_channel_ID_type>> stream_channel_ids, std::vector<std::vector<flash_chip_ID_type>> stream_chip_ids,
			std::vector<std::vector<flash_die_ID_type>> stream_die_ids, std::vector<std::vector<flash_plane_ID_type>> stream_plane_ids,
			unsigned int Block_no_per_plane, unsigned int Page_no_per_block, unsigned int SectorsPerPage, unsigned int PageSizeInBytes,
			double Overprovisioning_ratio, CMT_Sharing_Mode sharing_mode = CMT_Sharing_Mode::SHARED, bool fold_large_addresses = true);
    ~Address_Mapping_Unit_Page_Level_And_RAID();
    PPA_type Get_ppa(const bool ideal_mapping, const stream_id_type stream_id, const LPA_type lpa);
    LPA_type Get_And_Create_RAIDID(const stream_id_type stream_id, const LPA_type lpn);
    stream_id_type get_RAID_stream(int channel);
    
    page_status_type Get_page_status(const bool ideal_mapping, const stream_id_type stream_id, const LPA_type lpa);
   
    /*GC*/
    void Get_RAID_mapping_info_for_gc(const stream_id_type stream_id, const MVPN_type mvpn, PPA_type& mppa, sim_time_type& timestamp);
    void Allocate_new_page_for_gc(NVM_Transaction_Flash_WR* transaction, bool is_translation_page);

    void Set_barrier_for_accessing_physical_block(const NVM::FlashMemory::Physical_Page_Address& block_address);
    void Set_barrier_for_accessing_RAID(stream_id_type stream_id, LPA_type lpa);
    void Remove_barrier_for_accessing_RAID(stream_id_type stream_id, LPA_type lpa);

    void Semilate_page(const stream_id_type stream_id, const LPA_type lpa);

    unsigned long long get_Semilate_page_count(const stream_id_type stream_id, const LPA_type lpa);
    unsigned long long get_Not_Semilate_page_count(const stream_id_type stream_id, const LPA_type lpa);

    std::vector<PPA_type> get_Semilate_pages(const stream_id_type stream_id, const LPA_type lpa);
    std::vector<LPA_type> get_Not_Semilate_pages(const stream_id_type stream_id, const LPA_type lpa);
};

}


#endif