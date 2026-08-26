#include <cmath>
#include <assert.h>
#include <stdexcept>

#include "Address_Mapping_Unit_Page_Level_And_RAID.h"
#include "Address_Mapping_Unit_Page_Level.h"
#include "Stats.h"
#include "../utils/Logical_Address_Partitioning_Unit.h"
#include "SSD_Defs.h"
#include "Data_Cache_Manager_Flash_RAID.h"

namespace SSD_Components{

AddressMappingDomain_RAID::AddressMappingDomain_RAID(unsigned int cmt_capacity, unsigned int cmt_entry_size, unsigned int no_of_translation_entries_per_page,
			Cached_Mapping_Table* CMT,
			Flash_Plane_Allocation_Scheme_Type PlaneAllocationScheme,
			flash_channel_ID_type* channel_ids, unsigned int channel_no, flash_chip_ID_type* chip_ids, unsigned int chip_no,
			flash_die_ID_type* die_ids, unsigned int die_no, flash_plane_ID_type* plane_ids, unsigned int plane_no,
			PPA_type total_physical_sectors_no, LHA_type total_logical_sectors_no, unsigned int sectors_no_per_page, int raidGourpCount)
            :AddressMappingDomain(cmt_capacity, cmt_entry_size,no_of_translation_entries_per_page,
			CMT,
			PlaneAllocationScheme,
			channel_ids, channel_no, chip_ids, chip_no,
			die_ids, die_no, plane_ids, plane_no,
			total_physical_sectors_no, total_logical_sectors_no, sectors_no_per_page), channelToken(0),
            nowRAIDID(raidGourpCount, 0){
                
            }

Address_Mapping_Unit_Page_Level_And_RAID::Address_Mapping_Unit_Page_Level_And_RAID(const sim_object_id_type& id, FTL* ftl, NVM_PHY_ONFI* flash_controller, Flash_Block_Manager_Base* block_manager,
			bool ideal_mapping_table, unsigned int cmt_capacity_in_byte, Flash_Plane_Allocation_Scheme_Type PlaneAllocationScheme,
			unsigned int ConcurrentStreamNo,
			unsigned int ChannelCount, unsigned int chip_no_per_channel, unsigned int DieNoPerChip, unsigned int PlaneNoPerDie,
			std::vector<std::vector<flash_channel_ID_type>> stream_channel_ids, std::vector<std::vector<flash_chip_ID_type>> stream_chip_ids,
			std::vector<std::vector<flash_die_ID_type>> stream_die_ids, std::vector<std::vector<flash_plane_ID_type>> stream_plane_ids,
			unsigned int Block_no_per_plane, unsigned int Page_no_per_block, unsigned int SectorsPerPage, unsigned int PageSizeInBytes,
			double Overprovisioning_ratio, CMT_Sharing_Mode sharing_mode, bool fold_large_addresses)
            :Address_Mapping_Unit_Page_Level(id, ftl, flash_controller, block_manager,
			ideal_mapping_table, cmt_capacity_in_byte, PlaneAllocationScheme,
			ConcurrentStreamNo,
			ChannelCount, chip_no_per_channel, DieNoPerChip, PlaneNoPerDie,
			stream_channel_ids, stream_chip_ids,
			stream_die_ids, stream_plane_ids,
			Block_no_per_plane, Page_no_per_block,SectorsPerPage, PageSizeInBytes,
			Overprovisioning_ratio, sharing_mode, fold_large_addresses),
            checkPosRAID(channel_count / (STRIPE_DATA_NUM + 1), 0),
            RAID2LPA(channel_count / (STRIPE_DATA_NUM + 1)),
            chipToken(channel_count, 0),
            dieToken(channel_count, 0),
            planeToken(channel_count, 0){
    _my_instance = this;
    for (unsigned int domainID = 0; domainID < no_of_input_streams; domainID++){
        delete domains[domainID];
    }
    delete[] domains;
    domains = reinterpret_cast<AddressMappingDomain**>(new AddressMappingDomain_RAID*[no_of_input_streams]);
    flash_channel_ID_type* channel_ids = NULL;
    flash_channel_ID_type* chip_ids = NULL;
    flash_channel_ID_type* die_ids = NULL;
    flash_channel_ID_type* plane_ids = NULL;
    for (unsigned int domainID = 0; domainID < no_of_input_streams; domainID++) {
        /* Since we want to have the same mapping table entry size for all streams, the entry size
        *  is calculated at this level and then pass it to the constructors of mapping domains
        * entry size = sizeOf(lpa) + sizeOf(ppn) + sizeOf(bit vector that shows written sectors of a page)
        */
        CMT_entry_size = (unsigned int)std::ceil(((2 * std::log2(total_physical_pages_no)) + sector_no_per_page) / 8);
        //In GTD we do not need to store lpa
        GTD_entry_size = (unsigned int)std::ceil((std::log2(total_physical_pages_no) + sector_no_per_page) / 8);
        no_of_translation_entries_per_page = (SectorsPerPage * SECTOR_SIZE_IN_BYTE) / GTD_entry_size;

        Cached_Mapping_Table* sharedCMT = NULL;
        unsigned int per_stream_cmt_capacity = 0;
        cmt_capacity = cmt_capacity_in_byte / CMT_entry_size;
        switch (sharing_mode) {
            case CMT_Sharing_Mode::SHARED:
                per_stream_cmt_capacity = cmt_capacity;
                sharedCMT = new Cached_Mapping_Table(cmt_capacity);
                break;
            case CMT_Sharing_Mode::EQUAL_SIZE_PARTITIONING:
                per_stream_cmt_capacity = cmt_capacity / no_of_input_streams;
                break;
        }


        channel_ids = new flash_channel_ID_type[stream_channel_ids[domainID].size()];
        for (unsigned int i = 0; i < stream_channel_ids[domainID].size(); i++) {
            if (stream_channel_ids[domainID][i] < channel_count) {
                channel_ids[i] = stream_channel_ids[domainID][i];
            } else {
                PRINT_ERROR("Invalid channel ID specified for I/O flow " << domainID);
            }
        }

        chip_ids = new flash_channel_ID_type[stream_chip_ids[domainID].size()];
        for (unsigned int i = 0; i < stream_chip_ids[domainID].size(); i++) {
            if (stream_chip_ids[domainID][i] < chip_no_per_channel) {
                chip_ids[i] = stream_chip_ids[domainID][i];
            } else {
                PRINT_ERROR("Invalid chip ID specified for I/O flow " << domainID);
            }
        }

        die_ids = new flash_channel_ID_type[stream_die_ids[domainID].size()];
        for (unsigned int i = 0; i < stream_die_ids[domainID].size(); i++) {
            if (stream_die_ids[domainID][i] < die_no_per_chip) {
                die_ids[i] = stream_die_ids[domainID][i];
            } else {
                PRINT_ERROR("Invalid die ID specified for I/O flow " << domainID);
            }
        }

        plane_ids = new flash_channel_ID_type[stream_plane_ids[domainID].size()];
        for (unsigned int i = 0; i < stream_plane_ids[domainID].size(); i++) {
            if (stream_plane_ids[domainID][i] < plane_no_per_die) {
                plane_ids[i] = stream_plane_ids[domainID][i];
            } else {
                PRINT_ERROR("Invalid plane ID specified for I/O flow " << domainID);
            }
        }
       
        domains[domainID] = new AddressMappingDomain_RAID(per_stream_cmt_capacity, CMT_entry_size, no_of_translation_entries_per_page,
            sharedCMT,
            PlaneAllocationScheme,
            channel_ids, (unsigned int)(stream_channel_ids[domainID].size()), chip_ids, (unsigned int)(stream_chip_ids[domainID].size()), die_ids, 
            (unsigned int)(stream_die_ids[domainID].size()), plane_ids, (unsigned int)(stream_plane_ids[domainID].size()),
            Utils::Logical_Address_Partitioning_Unit::PDA_count_allocate_to_flow(domainID), Utils::Logical_Address_Partitioning_Unit::LHA_count_allocate_to_flow_from_device_view(domainID),
            sector_no_per_page, channel_count / (STRIPE_DATA_NUM + 1));
        delete[] channel_ids;
        delete[] chip_ids;
        delete[] die_ids;
        delete[] plane_ids;
    }
}

Address_Mapping_Unit_Page_Level_And_RAID::~Address_Mapping_Unit_Page_Level_And_RAID(){
    std::vector<int> parityCounts(channel_count, 0);
    for(auto &RAIDs : RAID2LPA){
        for(auto &RAID : RAIDs){
            PPA_type ppa = RAID[STRIPE_DATA_NUM];
            NVM::FlashMemory::Physical_Page_Address address;
            Convert_ppa_to_address(ppa, address);
            parityCounts[address.ChannelID]++;
        }
    }
    for(auto &parityCount : parityCounts){
        std::cout << parityCount << "\t";
    }
    std::cout << std::endl;
}

PPA_type Address_Mapping_Unit_Page_Level_And_RAID::Get_ppa(const bool ideal_mapping, const stream_id_type stream_id, const LPA_type lpa){
    if(stream_id > RAID_STREAN - RAID2LPA.size()){
        return RAID2LPA[RAID_STREAN - stream_id][lpa][STRIPE_DATA_NUM];
    }
    return domains[stream_id]->Get_ppa(ideal_mapping, stream_id, lpa);
}

void Address_Mapping_Unit_Page_Level_And_RAID::pre_translate_lpa_to_ppa(const stream_id_type stream_id, const LPA_type lpn, const int channel){
    PPA_type ppa = page_no_per_channel * channel;
    
    if(stream_id > RAID_STREAN - RAID2LPA.size()){
        RAID2LPA[RAID_STREAN - stream_id][lpn][STRIPE_DATA_NUM] = ppa;
    }else{
        domains[stream_id]->Update_mapping_info(true, stream_id, lpn, ppa, UNWRITTEN_LOGICAL_PAGE);
    }
}


stream_id_type Address_Mapping_Unit_Page_Level_And_RAID::get_RAID_stream(int channel){
    return RAID_STREAN - (channel / (STRIPE_DATA_NUM + 1));
}

LPA_type Address_Mapping_Unit_Page_Level_And_RAID::Get_And_Create_RAIDID(const stream_id_type stream_id, const LPA_type lpn){
    LPA_type key = LPN_TO_UNIQUE_KEY(stream_id, lpn);
    
    while(LPA2RAID.find(key) == LPA2RAID.end()){
        AddressMappingDomain_RAID *domain = reinterpret_cast<AddressMappingDomain_RAID*>(domains[stream_id]);
        int raidOffset = (domain->Channel_ids[domain->channelToken]) / (STRIPE_DATA_NUM + 1);
        //std::cout << raidOffset << std::endl;
        int raidid = static_cast<size_t>(UNIQUE_KEY_TO_LPN(RAID_STREAN - raidOffset, (domain->nowRAIDID[raidOffset])));
        
        if(raidid == RAID2LPA[raidOffset].size()){
            RAID2LPA[raidOffset].emplace_back(STRIPE_INFO_LEN, NO_LPA);
            int checkChannel = checkPosRAID[raidOffset] + (STRIPE_DATA_NUM + 1) * raidOffset;
            if(++checkPosRAID[raidOffset] == (STRIPE_DATA_NUM + 1)){
                checkPosRAID[raidOffset] = 0;
            }

            RAID2LPA[raidOffset][raidid][STRIPE_DATA_NUM + 1] = UNWRITTEN_LOGICAL_PAGE;
            RAID2LPA[raidOffset][raidid][STRIPE_DATA_NUM + 2] = 0;
            pre_translate_lpa_to_ppa(RAID_STREAN - raidOffset, RAID2LPA[raidOffset].size() - 1, checkChannel);
        }
        if(raidid >= RAID2LPA[raidOffset].size()){
            throw std::string("raidid >= RAID2LPA[raidOffset].size() in Address_Mapping_Unit_Page_Level_And_RAID::Get_And_Create_RAIDID");
        }
        
        PPA_type checkPPA = Get_ppa(true, static_cast<short>(RAID_STREAN - raidOffset), static_cast<LPA_type>(raidid));
        NVM::FlashMemory::Physical_Page_Address checkAddress;
        Convert_ppa_to_address(checkPPA, checkAddress); 
        int preDataChannel = domain->Channel_ids[domain->channelToken];
        
        
        if(++domain->channelToken % (STRIPE_DATA_NUM + 1) == 0){
            domain->nowRAIDID[raidOffset]++;
            if(domain->channelToken == domain->Channel_no)
                domain->channelToken = 0;
        }
        if(preDataChannel != checkAddress.ChannelID){
            pre_translate_lpa_to_ppa(stream_id, lpn, preDataChannel);
            int i = 0;
            for(; i < STRIPE_DATA_NUM; i++){
                if(RAID2LPA[raidOffset][raidid][i] == NO_LPA){
                    break;
                }
            }
           
            if(i == STRIPE_DATA_NUM){
                throw std::string("i == STRIPE_DATA_NUM in Address_Mapping_Unit_Page_Level_And_RAID::Get_And_Create_RAIDID");
            }
            RAID2LPA[raidOffset][raidid][i] = key;
            LPA2RAID[key] = LPN_TO_UNIQUE_KEY(RAID_STREAN - raidOffset, raidid);
            break;
        }
    }
    LPA_type raidLPA = LPA2RAID[key];
    stream_id_type stream = (raidLPA & (NO_LPA << 56)) >> 56;
    raidLPA = UNIQUE_KEY_TO_LPN(stream, raidLPA);
    int i = 0;
    for(; i < STRIPE_DATA_NUM; i++){
        if(RAID2LPA[RAID_STREAN - stream][raidLPA][i] == key){
            break;
        }
    }

    if(i == STRIPE_DATA_NUM){
        throw "if(i == STRIPE_DATA_NUM) ";
    }
    return LPA2RAID[key];
}



page_status_type Address_Mapping_Unit_Page_Level_And_RAID::Get_page_status(const bool ideal_mapping, const stream_id_type stream_id, const LPA_type lpa){
    if(stream_id > RAID_STREAN - RAID2LPA.size()){
        return RAID2LPA[RAID_STREAN - stream_id][lpa][STRIPE_DATA_NUM + 1];
    }
    return domains[stream_id]->Get_page_status(ideal_mapping, stream_id, lpa);
}

void Address_Mapping_Unit_Page_Level_And_RAID::Set_barrier_for_accessing_RAID(stream_id_type stream_id, LPA_type lpa){
    LPA_type key = LPN_TO_UNIQUE_KEY(stream_id, lpa);
    auto itr =Locked_RAIDs.find(key);
    if (itr != Locked_RAIDs.end()) {
        PRINT_ERROR("Illegal operation: Locking an MVPN that has already been locked!");
    }
    Locked_RAIDs.insert(key);
}

bool Address_Mapping_Unit_Page_Level_And_RAID::is_lpa_locked_for_gc(stream_id_type stream_id, LPA_type lpa){
    
    if(stream_id > RAID_STREAN - RAID2LPA.size()){
        LPA_type key = LPN_TO_UNIQUE_KEY(stream_id, lpa);
        return Locked_RAIDs.find(key) != Locked_RAIDs.end();
    }else{
        return domains[stream_id]->Locked_LPAs.find(lpa) != domains[stream_id]->Locked_LPAs.end();
    }
}

void Address_Mapping_Unit_Page_Level_And_RAID::manage_user_transaction_facing_barrier(NVM_Transaction_Flash* transaction){
    if(transaction->Stream_id > RAID_STREAN - RAID2LPA.size()){
        LPA_type key = LPN_TO_UNIQUE_KEY(transaction->Stream_id, transaction->LPA);
        std::pair<LPA_type, NVM_Transaction_Flash*> entry(key, transaction);
        if (transaction->Type == Transaction_Type::READ){
            Read_transactions_behind_RAID_barrier.insert(entry);
        }else{
            Write_transactions_behind_RAID_barrier.insert(entry);
        }   
    }else{
        
        std::pair<LPA_type, NVM_Transaction_Flash*> entry(transaction->LPA, transaction);
        if (transaction->Type == Transaction_Type::READ) {
            domains[transaction->Stream_id]->Read_transactions_behind_LPA_barrier.insert(entry);
        } else {
            domains[transaction->Stream_id]->Write_transactions_behind_LPA_barrier.insert(entry);
        }
    }
    
}

void Address_Mapping_Unit_Page_Level_And_RAID::Remove_barrier_for_accessing_RAID(stream_id_type stream_id, LPA_type lpa){
    std::list<NVM_Transaction*> transactions;
    LPA_type key = LPN_TO_UNIQUE_KEY(stream_id, lpa);
    auto itr = Locked_RAIDs.find(key);
    if (itr == Locked_RAIDs.end()) {
        PRINT_MESSAGE(lpa << " ")
        PRINT_ERROR("Illegal operation: Unlocking an RAID that has not been locked!");
    }
    Locked_RAIDs.erase(itr);

    auto read_tr = Read_transactions_behind_RAID_barrier.find(key);
    while(read_tr != Read_transactions_behind_RAID_barrier.end()){
        NVM_Transaction* transaction = dynamic_cast<NVM_Transaction*>((*read_tr).second);
        transactions.push_back(transaction);

        Read_transactions_behind_RAID_barrier.erase(read_tr);
        read_tr = Read_transactions_behind_RAID_barrier.find(key);
    }

    auto write_tr = Write_transactions_behind_RAID_barrier.find(key);
    while (write_tr != Write_transactions_behind_RAID_barrier.end()){
        NVM_Transaction* transaction = dynamic_cast<NVM_Transaction*>((*write_tr).second);
		transactions.push_back(transaction);

        Write_transactions_behind_RAID_barrier.erase(write_tr);
        write_tr = Write_transactions_behind_RAID_barrier.find(key);
    }
    if(transactions.size() > 0){
        Translate_lpa_to_ppa_and_dispatch(transactions);
    }
}

void Address_Mapping_Unit_Page_Level_And_RAID::Set_barrier_for_accessing_physical_block(const NVM::FlashMemory::Physical_Page_Address& block_address){
    //The LPAs are actually not known until they are read one-by-one from flash storage. But, to reduce MQSim's complexity, we assume that LPAs are stored in DRAM and thus no read from flash storage is needed.
    Block_Pool_Slot_Type* block = &(block_manager->plane_manager[block_address.ChannelID][block_address.ChipID][block_address.DieID][block_address.PlaneID].Blocks[block_address.BlockID]);
    NVM::FlashMemory::Physical_Page_Address addr(block_address);
    int barriercount = 0;
    //std::cout << "Set_barrier_for_accessing_physical_block" << std::endl;
    for (flash_page_ID_type pageID = 0; pageID < block->Current_page_write_index; pageID++) {
        addr.PageID = pageID;
        LPA_type lpa = flash_controller->Get_metadata(addr.ChannelID, addr.ChipID, addr.DieID, addr.PlaneID, addr.BlockID, addr.PageID);
        
        if (block_manager->Is_page_valid(block, pageID)) {
            barriercount++;
            
            if(block->Holds_RAID_data){
                stream_id_type stream = get_RAID_stream(block_address.ChannelID);
                PPA_type ppa = RAID2LPA[RAID_STREAN - stream][lpa][STRIPE_DATA_NUM];
                if(ppa != Convert_address_to_ppa(addr)){
                    PRINT_ERROR("Inconsistency in the global translation directory when locking an RAID!")
                }
                Set_barrier_for_accessing_RAID(stream, lpa);
            }else if (block->Holds_mapping_data) {
                MVPN_type mpvn = (MVPN_type)flash_controller->Get_metadata(addr.ChannelID, addr.ChipID, addr.DieID, addr.PlaneID, addr.BlockID, addr.PageID);
                if (domains[block->Stream_id]->GlobalTranslationDirectory[mpvn].MPPN != Convert_address_to_ppa(addr)) {
                    PRINT_ERROR("Inconsistency in the global translation directory when locking an MPVN!")
                }else{
                    Set_barrier_for_accessing_mvpn(block->Stream_id, mpvn);
                }
            } else {
                //std::cout << block->Stream_id << " " << lpa  << std::endl;
                
                LPA_type ppa = domains[block->Stream_id]->GlobalMappingTable[lpa].PPA;
                if(ppa != Convert_address_to_ppa(addr)){
                    PRINT_ERROR("ppa != Convert_address_to_ppa(addr)"<< lpa << " " << ppa <<" " <<Convert_address_to_ppa(addr))
                }
                if (domains[block->Stream_id]->CMT->Exists(block->Stream_id, lpa)) {
                    ppa = domains[block->Stream_id]->CMT->Retrieve_ppa(block->Stream_id, lpa);
                }
                if (ppa != Convert_address_to_ppa(addr)) {
                    PRINT_ERROR("Inconsistency in the global mapping table when locking an LPA!")
                }
                Set_barrier_for_accessing_lpa(block->Stream_id, lpa);
            }
        }
    }
}

bool Address_Mapping_Unit_Page_Level_And_RAID::Mapping_entry_accessible(const bool ideal_mapping, const stream_id_type stream_id, const LPA_type lpa){
    if (ideal_mapping || stream_id > RAID_STREAN - RAID2LPA.size()) {
        return true;
    } else {
        return domains[stream_id]->Mapping_entry_accessible(ideal_mapping_table, stream_id, lpa);
    }
}

bool Address_Mapping_Unit_Page_Level_And_RAID::query_cmt(NVM_Transaction_Flash* transaction){
	stream_id_type stream_id = transaction->Stream_id;
	Stats::total_CMT_queries++;
	Stats::total_CMT_queries_per_stream[stream_id]++;
    
    if (Mapping_entry_accessible(ideal_mapping_table, stream_id, transaction->LPA))//Either limited or unlimited CMT
    {
        Stats::CMT_hits_per_stream[stream_id]++;
        Stats::CMT_hits++;
        if (transaction->Type == Transaction_Type::READ) {
            Stats::total_readTR_CMT_queries_per_stream[stream_id]++;
            Stats::total_readTR_CMT_queries++;
            Stats::readTR_CMT_hits_per_stream[stream_id]++;
            Stats::readTR_CMT_hits++;
        } else {
            //This is a write transaction
            Stats::total_writeTR_CMT_queries++;
            Stats::total_writeTR_CMT_queries_per_stream[stream_id]++;
            Stats::writeTR_CMT_hits++;
            Stats::writeTR_CMT_hits_per_stream[stream_id]++;
        }
        
        if (translate_lpa_to_ppa(stream_id, transaction)) {
            return true;
        } else {
            mange_unsuccessful_translation(transaction);
            return false;
        }
    } else {//Limited CMT
        //Maybe we can catch mapping data from an on-the-fly write back request
        

        return false;
    }
	
}

bool Address_Mapping_Unit_Page_Level_And_RAID::translate_lpa_to_ppa(stream_id_type streamID, NVM_Transaction_Flash* transaction)
{
    PPA_type ppa = Get_ppa(ideal_mapping_table, streamID, transaction->LPA);
        
        
    if(streamID > RAID_STREAN - RAID2LPA.size() && RAID2LPA[RAID_STREAN - streamID].size() <= transaction->LPA){
        throw "RAID2LPA[stream].size() <= raidid";
    }

    if (transaction->Type == Transaction_Type::READ) {
        
        if (ppa == NO_PPA) {
            ppa = online_create_entry_for_reads(transaction->LPA, streamID, transaction->Address, ((NVM_Transaction_Flash_RD*)transaction)->read_sectors_bitmap);
            block_manager->Program_transaction_serviced(transaction->Address);
        }
        transaction->PPA = ppa;
        
        Convert_ppa_to_address(transaction->PPA, transaction->Address);
        block_manager->Read_transaction_issued(transaction->Address);
        transaction->Physical_address_determined = true;
        
        return true;
    } else {//This is a write transaction
        
        allocate_plane_for_user_write((NVM_Transaction_Flash_WR*)transaction);
        //there are too few free pages remaining only for GC
        if (ftl->GC_and_WL_Unit->Stop_servicing_tlc_writes(transaction->Address)){
            return false;
        }
        Semilate_page(transaction->Stream_id, transaction->LPA);
        allocate_page_in_plane_for_user_write((NVM_Transaction_Flash_WR*)transaction, false);
        transaction->Physical_address_determined = true;
        
        return true;
    }
}

std::vector<PPA_type> Address_Mapping_Unit_Page_Level_And_RAID::get_Semilate_pages(const stream_id_type stream_id, const LPA_type lpa){
    std::vector<LPA_type> ret;
    for(int i = 0; i < STRIPE_DATA_NUM; ++i){
        if(RAID2LPA[RAID_STREAN - stream_id][lpa][STRIPE_INFO_LEN - 1 - i] != NO_PPA){
            ret.emplace_back(RAID2LPA[RAID_STREAN - stream_id][lpa][STRIPE_INFO_LEN - 1 - i]);
        }
    }
    return ret;
}

std::vector<LPA_type> Address_Mapping_Unit_Page_Level_And_RAID::get_Not_Semilate_pages(const stream_id_type stream_id, const LPA_type lpa){
    std::vector<LPA_type> ret;
    for(int i = 0; i < STRIPE_DATA_NUM; ++i){
        if(RAID2LPA[RAID_STREAN - stream_id][lpa][STRIPE_INFO_LEN - 1 - i] == NO_PPA){
            ret.emplace_back(RAID2LPA[RAID_STREAN - stream_id][lpa][i]);
        }
    }
    return ret;
}

unsigned long long Address_Mapping_Unit_Page_Level_And_RAID::get_Not_Semilate_page_count(const stream_id_type stream_id, const LPA_type lpa){
    unsigned long long ret = 0;
    for(int i = 0; i < STRIPE_DATA_NUM; ++i){
        if(RAID2LPA[RAID_STREAN - stream_id][lpa][STRIPE_INFO_LEN - 1 - i] == NO_PPA){
            ret++;
        }
    }
    return ret;
}

unsigned long long Address_Mapping_Unit_Page_Level_And_RAID::get_Semilate_page_count(const stream_id_type stream_id, const LPA_type lpa){
    unsigned long long ret = 0;
    for(int i = 0; i < STRIPE_DATA_NUM; ++i){
        if(RAID2LPA[RAID_STREAN - stream_id][lpa][STRIPE_INFO_LEN - 1 - i] != NO_PPA){
            ret++;
        }
    }
    return ret;
}

void Address_Mapping_Unit_Page_Level_And_RAID::Semilate_page(const stream_id_type stream_id, const LPA_type lpa){
    if(stream_id > RAID_STREAN - RAID2LPA.size()){
        for(int i = 0; i < STRIPE_DATA_NUM; i++){
            if(RAID2LPA[RAID_STREAN - stream_id][lpa][STRIPE_INFO_LEN - 1 - i] != NO_LPA){
                NVM::FlashMemory::Physical_Page_Address addrInter;
                Convert_ppa_to_address(RAID2LPA[RAID_STREAN - stream_id][lpa][STRIPE_INFO_LEN - 1 - i], addrInter);
                stream_id_type stream = (RAID2LPA[RAID_STREAN - stream_id][lpa][i] & (NO_LPA << 56)) >> 56;
                block_manager->Insemilate_page_in_block(stream, addrInter);
                RAID2LPA[RAID_STREAN - stream_id][lpa][STRIPE_INFO_LEN - 1 - i] = NO_LPA;
            }
        }
    }else {
        LPA_type key = LPN_TO_UNIQUE_KEY(stream_id, lpa);
        LPA_type raidLPA = Get_And_Create_RAIDID(stream_id, lpa);
        stream_id_type stream = (raidLPA & (NO_LPA << 56)) >> 56;
        raidLPA = UNIQUE_KEY_TO_LPN(stream, raidLPA);
        page_status_type  page_status = Get_page_status(true, stream, raidLPA);
        Data_Cache_Manager_Flash_RAID *dcm = dynamic_cast<Data_Cache_Manager_Flash_RAID*>(ftl->Data_cache_manager);
		
        if(page_status == UNWRITTEN_LOGICAL_PAGE || !dcm->is_exist(stream, raidLPA)){
            return;
        }

        
        int i = 0;
        for(; i < STRIPE_DATA_NUM; i++){
            if(key == RAID2LPA[RAID_STREAN - stream][raidLPA][i]){
                break;
            }
        }
        if(i == STRIPE_DATA_NUM){
            throw "i == STRIPE_DATA_NUM ";
        }
        if(RAID2LPA[RAID_STREAN - stream][raidLPA][STRIPE_INFO_LEN - 1 - i] == NO_LPA){
            PPA_type ppa = Get_ppa(ideal_mapping_table, stream_id, lpa);
            page_status_type prev_page_status = Get_page_status(ideal_mapping_table, stream_id, lpa);
            if(ppa != NO_PPA && prev_page_status != UNWRITTEN_LOGICAL_PAGE){
                NVM::FlashMemory::Physical_Page_Address addr;
                Convert_ppa_to_address(ppa, addr);
                RAID2LPA[RAID_STREAN - stream][raidLPA][STRIPE_INFO_LEN - 1 - i] = ppa;
                block_manager->Semilate_page_in_block(stream_id, addr);
            }
        }else{
            return;
        }
        
        for(i = 0; i < STRIPE_DATA_NUM; i++){
            if(RAID2LPA[RAID_STREAN - stream][raidLPA][STRIPE_INFO_LEN - i - 1] == NO_LPA){
                break;
            }
        }
        if(i == STRIPE_DATA_NUM && !is_lpa_locked_for_gc(stream, raidLPA)){
            NVM::FlashMemory::Physical_Page_Address addrInter;
            for(int j = 0; j < STRIPE_DATA_NUM; j++){
                if(RAID2LPA[RAID_STREAN - stream][raidLPA][STRIPE_INFO_LEN - 1 - j] != NO_LPA){
                    Convert_ppa_to_address(RAID2LPA[RAID_STREAN - stream][raidLPA][STRIPE_INFO_LEN - 1 - j], addrInter);
                    stream_id_type stream_lpn = (RAID2LPA[RAID_STREAN - stream][raidLPA][j] & (NO_LPA << 56)) >> 56;
                    block_manager->Insemilate_page_in_block(stream_lpn, addrInter);
                    RAID2LPA[RAID_STREAN - stream][raidLPA][STRIPE_INFO_LEN - 1 - j] = NO_LPA;
                }
            }
            PPA_type ppa = Get_ppa(true, stream, raidLPA);
            Convert_ppa_to_address(ppa, addrInter);
            block_manager->Invalidate_page_in_block(RAID_STREAN, addrInter);
            Update_mapping_info(true, stream, raidLPA, ppa, UNWRITTEN_LOGICAL_PAGE);
        }
    }
}

void Address_Mapping_Unit_Page_Level_And_RAID::allocate_page_in_plane_for_user_write(NVM_Transaction_Flash_WR* transaction, bool is_for_gc)
{
    AddressMappingDomain* domain = domains[transaction->Stream_id];
    PPA_type old_ppa = Get_ppa(ideal_mapping_table, transaction->Stream_id, transaction->LPA);
    page_status_type prev_page_status = Get_page_status(ideal_mapping_table, transaction->Stream_id, transaction->LPA);
    
    if (old_ppa == NO_PPA || prev_page_status == UNWRITTEN_LOGICAL_PAGE)  /*this is the first access to the logical page*/
    {
        if (is_for_gc) {
            std::cout << "this w " << transaction->Stream_id << std::endl;
            PRINT_ERROR("Unexpected mapping table status in allocate_page_in_plane_for_user_write function for a GC/WL write!")
        }
    } else {
        if (is_for_gc) {
            NVM::FlashMemory::Physical_Page_Address addr;
            Convert_ppa_to_address(old_ppa, addr);
			block_manager->Invalidate_page_in_block(transaction->Stream_id, addr);
            page_status_type page_status_in_cmt = Get_page_status(ideal_mapping_table, transaction->Stream_id, transaction->LPA);
            
            if (page_status_in_cmt != transaction->write_sectors_bitmap){
                PRINT_ERROR("Unexpected mapping table status in allocate_page_in_plane_for_user_write for a GC/WL write!")
            } 
        } else {
            page_status_type prev_page_status = Get_page_status(ideal_mapping_table, transaction->Stream_id, transaction->LPA);
            page_status_type status_intersection = transaction->write_sectors_bitmap & prev_page_status;
            //check if an update read is required
            
            if (status_intersection == prev_page_status) {
                NVM::FlashMemory::Physical_Page_Address addr;
                Convert_ppa_to_address(old_ppa, addr);
				block_manager->Invalidate_page_in_block(
					transaction->Stream_id, addr, true);
            } else {
                data_timestamp_type TimeStamp;
                page_status_type read_pages_bitmap = status_intersection ^ prev_page_status;
                if(transaction->Stream_id > RAID_STREAN - RAID2LPA.size()){
                    TimeStamp = RAID2LPA[RAID_STREAN - transaction->Stream_id][transaction->LPA][STRIPE_DATA_NUM + 2];
                }else{
                    TimeStamp = domain->GlobalMappingTable[transaction->LPA].TimeStamp;
                }
                
                if(transaction->hasRelatedRead == false){
                    
                    readCount++;
                    NVM_Transaction_Flash_RD *update_read_tr = new NVM_Transaction_Flash_RD(transaction->Source, transaction->Stream_id,
                        count_sector_no_from_status_bitmap(read_pages_bitmap) * SECTOR_SIZE_IN_BYTE, transaction->LPA, old_ppa, transaction->UserIORequest,
                        transaction->Content, transaction, read_pages_bitmap, TimeStamp);
                    Convert_ppa_to_address(old_ppa, update_read_tr->Address);
                    block_manager->Read_transaction_issued(update_read_tr->Address);//Inform block manager about a new transaction as soon as the transaction's target address is determined
					block_manager->Invalidate_page_in_block(
						transaction->Stream_id, update_read_tr->Address, true);
                    transaction->RelatedRead = update_read_tr;
                    // if(readCount >= 1917930){
                    //     std::cout << "tttttr " << update_read_tr << " " << readCount << " " << update_read_tr->LPA<< std::endl;
                    // }
                }else{
                    NVM::FlashMemory::Physical_Page_Address addr;
                    Convert_ppa_to_address(old_ppa, addr);
					block_manager->Invalidate_page_in_block(
						transaction->Stream_id, addr, true);
                }
            }
        }
    }

    /*The following lines should not be ordered with respect to the block_manager->Invalidate_page_in_block
    * function call in the above code blocks. Otherwise, GC may be invoked (due to the call to Allocate_block_....) and
    * may decide to move a page that is just invalidated.*/
    
    
    if(transaction->Stream_id > RAID_STREAN - RAID2LPA.size()){ 
        block_manager->Allocate_block_and_page_in_plane_for_RAID_write(transaction->Stream_id, transaction->Address, is_for_gc);
    }else if (is_for_gc) {
        block_manager->Allocate_block_and_page_in_plane_for_gc_write(transaction->Stream_id, transaction->Address);
    } else {
        block_manager->Allocate_block_and_page_in_plane_for_user_write(transaction->Stream_id, transaction->Address);
    }
    
    
    transaction->PPA = Convert_address_to_ppa(transaction->Address);
    Update_mapping_info(ideal_mapping_table, transaction->Stream_id, transaction->LPA, transaction->PPA,
        ((NVM_Transaction_Flash_WR*)transaction)->write_sectors_bitmap | Get_page_status(ideal_mapping_table, transaction->Stream_id, transaction->LPA));
}



void Address_Mapping_Unit_Page_Level_And_RAID::Update_mapping_info(const bool ideal_mapping, const stream_id_type stream_id, const LPA_type lpa, const PPA_type ppa, const page_status_type page_status_bitmap){
    
    if(stream_id > RAID_STREAN - RAID2LPA.size()){
        RAID2LPA[RAID_STREAN - stream_id][lpa][STRIPE_DATA_NUM] = ppa;
        RAID2LPA[RAID_STREAN - stream_id][lpa][STRIPE_DATA_NUM + 1] = page_status_bitmap;
        RAID2LPA[RAID_STREAN - stream_id][lpa][STRIPE_DATA_NUM + 2] = CurrentTimeStamp;
    }else{
        domains[stream_id]->Update_mapping_info(true, stream_id, lpa, ppa, page_status_bitmap);
    }
}


void Address_Mapping_Unit_Page_Level_And_RAID::Get_RAID_mapping_info_for_gc(const stream_id_type stream_id, const MVPN_type raidid, PPA_type& ppa, sim_time_type& timestamp){
    ppa = RAID2LPA[RAID_STREAN - stream_id][raidid][STRIPE_DATA_NUM];
    timestamp = RAID2LPA[RAID_STREAN - stream_id][raidid][STRIPE_DATA_NUM + 1];
}


void Address_Mapping_Unit_Page_Level_And_RAID::Allocate_new_page_for_gc(NVM_Transaction_Flash_WR* transaction, bool is_translation_page){

    if(transaction->Stream_id > RAID_STREAN - RAID2LPA.size()){
        allocate_page_in_plane_for_user_write(transaction, true);
        transaction->Physical_address_determined = true;

        Update_mapping_info(ideal_mapping_table, transaction->Stream_id, transaction->LPA, transaction->PPA, transaction->write_sectors_bitmap);
    }else if (is_translation_page) {
        MPPN_type mppn = domains[transaction->Stream_id]->GlobalTranslationDirectory[transaction->LPA].MPPN;
        if (mppn == NO_MPPN) {
            PRINT_ERROR("Unexpected situation occured for gc write in Allocate_new_page_for_gc function!")
        }

        allocate_page_in_plane_for_translation_write(transaction, (MVPN_type)transaction->LPA, true);
        
        transaction->Physical_address_determined = true;
    } else {
        
        allocate_page_in_plane_for_user_write(transaction, true);
        transaction->Physical_address_determined = true;

        //the mapping entry should be updated
        stream_id_type stream_id = transaction->Stream_id;
        Stats::total_CMT_queries++;
        Stats::total_CMT_queries_per_stream[stream_id]++;

        //either limited or unlimited mapping
        if (domains[stream_id]->Mapping_entry_accessible(ideal_mapping_table, stream_id, transaction->LPA)) {
            Stats::CMT_hits++;
            Stats::CMT_hits_per_stream[stream_id]++;
            Stats::total_writeTR_CMT_queries++;
            Stats::total_writeTR_CMT_queries_per_stream[stream_id]++;
            Stats::writeTR_CMT_hits++;
            Stats::writeTR_CMT_hits_per_stream[stream_id]++;
            Update_mapping_info(ideal_mapping_table, stream_id, transaction->LPA, transaction->PPA, transaction->write_sectors_bitmap);
        }
    }
}

void Address_Mapping_Unit_Page_Level_And_RAID::allocate_plane(stream_id_type Stream_id, LPA_type lpn, NVM::FlashMemory::Physical_Page_Address& targetAddress){
    
    PPA_type ppa = Get_ppa(true, Stream_id, lpn);

    targetAddress.ChannelID = ppa / page_no_per_channel;
  
    targetAddress.ChipID = chipToken[targetAddress.ChannelID];
    targetAddress.DieID = dieToken[targetAddress.ChannelID];
    targetAddress.PlaneID = planeToken[targetAddress.ChannelID];

    if(++chipToken[targetAddress.ChannelID] == chip_no_per_channel){
        chipToken[targetAddress.ChannelID] = 0;
        if(++dieToken[targetAddress.ChannelID] == die_no_per_chip){
            dieToken[targetAddress.ChannelID] = 0;
            if(++planeToken[targetAddress.ChannelID] == plane_no_per_die){
                planeToken[targetAddress.ChannelID] = 0;
            }
        }
    }
}

void Address_Mapping_Unit_Page_Level_And_RAID::allocate_plane_for_user_write(NVM_Transaction_Flash_WR* transaction){
    if(!transaction->allocatedPlane)
        allocate_plane(transaction->Stream_id, transaction->LPA, transaction->Address);
    transaction->allocatedPlane = true;
}

PPA_type Address_Mapping_Unit_Page_Level_And_RAID::online_create_entry_for_reads(LPA_type lpa, const stream_id_type stream_id, NVM::FlashMemory::Physical_Page_Address& read_address, uint64_t read_sectors_bitmap)
{
    
    LPA_type raidKey = Get_And_Create_RAIDID(stream_id, lpa);
    stream_id_type stream = (raidKey & (NO_LPA << 56)) >> 56;
    LPA_type raidid = UNIQUE_KEY_TO_LPN(stream, raidKey);
    
    if(RAID2LPA[RAID_STREAN - stream].size() <= raidid){
        std::cout << stream << " "<<RAID2LPA[stream].size() << " " << raidid << std::endl;
        throw "RAID2LPA[stream].size() >= raidid";
    }

    if(Get_page_status(ideal_mapping_table, stream, raidid) == UNWRITTEN_LOGICAL_PAGE){
        NVM::FlashMemory::Physical_Page_Address raid_address;
        allocate_plane(stream, raidid, raid_address);

        block_manager->Allocate_block_and_page_in_plane_for_RAID_write(stream, raid_address, false);
        PPA_type ppa = Convert_address_to_ppa(raid_address);
        
        Update_mapping_info(ideal_mapping_table, stream, raidid, ppa, read_sectors_bitmap);
        flash_controller->Set_metadata(raid_address.ChannelID, raid_address.ChipID, raid_address.DieID, raid_address.PlaneID, raid_address.BlockID, raid_address.PageID, raidid);
        block_manager->Program_transaction_serviced(raid_address);
    }

    allocate_plane(stream_id, lpa, read_address);
    block_manager->Allocate_block_and_page_in_plane_for_user_write(stream_id, read_address);
    PPA_type ppa = Convert_address_to_ppa(read_address);
    Update_mapping_info(ideal_mapping_table, stream_id, lpa, ppa, read_sectors_bitmap);
    flash_controller->Set_metadata(read_address.ChannelID, read_address.ChipID, read_address.DieID, read_address.PlaneID, read_address.BlockID, read_address.PageID, lpa);
    
    return ppa;
}



}
