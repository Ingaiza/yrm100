#include <string.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include "/home/ingaiza/yrm_module/src/yrm100/include/yrm100/uhf_module_cmd.h"
#include "/home/ingaiza/yrm_module/src/yrm100/include/yrm100/uhf_module.hpp"


#define DELAY_MS 100
#define WAIT_TICK 4000 // max wait time in between each byte

static M100ResponseType setup_and_send_rx(M100Module* module, uint8_t* cmd, size_t cmd_length) {
    // std::cout<<"Initiallizing setup_and_send_rx"<<std::endl;

    if (!module || !module->serial || !module->serial->buffer_) 
    {
        std::cout<<"Invalid module or serial buffer"<<std::endl;
        return M100ValidationFail;
    }
    // printf("module is valid");
    // std::cout<<"module is valid"<<std::endl;
    // clear buffer
    if(!module->serial->buffer_->multi)
    {
        uhf_buffer_reset(module->serial->buffer_);
    }
    // Validation Checks
    size_t length = module->serial->send_wait(cmd, cmd_length,module->serial->buffer_->data);
    uint8_t* data = module->serial->buffer_->data;
    // check if size > 0
    if(!length) 
    {
        std::cout<<"Empty Response in setup_and_send_rx"<<std::endl;
        return M100EmptyResponse;
    }
    // check if data is valid
    if(data[0] != FRAME_START || data[length - 1] != FRAME_END)
    {
        std::cout<<"Validation Failed in setup_and_send_rx "<<std::endl;
        return M100ValidationFail;
    }
    // check if checksum is correct
    if(checksum(data + 1, length - 3) != data[length - 2])
    {
        std::cout<<"Checksum failed in setup_and_send_rx"<<std::endl;
        return M100ChecksumFail;
    }
    // std::cout<<"Setup and send rx completed "<<std::endl;
    return M100SuccessResponse;
}

static M100ResponseType setup_and_send_rx_multiple(M100Module* module, uint8_t* cmd, size_t cmd_length) 
{
    std::cout<<"Initializing setup_and_send_rx"<<std::endl;
    // clear buffer
    // for(int buffer=0;buffer < 5;buffer++)
    // {
    //     uhf_buffer_reset(module->serial->multi_buffer_[buffer]);
    // }
    Buffer* buffer_return_[5];
    for (int i = 0; i < 5; i++)  
    {
        // std::cout << "Starting buffer allocation for index " << i << std::endl;
        // Allocate the Buffer struct itself
        buffer_return_[i] = (Buffer*)malloc(sizeof(Buffer));
        if (buffer_return_[i] == nullptr) 
        {
            std::cerr << "Failed to allocate Buffer struct for index " << i << std::endl;
            // Clean up previously allocated buffers
            for (int j = 0; j < i; j++) 
            {
                if (buffer_return_[j] != nullptr) 
                {
                    if (buffer_return_[j]->data != nullptr) 
                    {
                        free(buffer_return_[j]->data);
                    }
                    free(buffer_return_[j]);
                }
            }
            return M100EmptyResponse;
        }

        // Allocate the data buffer
        buffer_return_[i]->data = (uint8_t*)malloc(256);
        if (buffer_return_[i]->data == nullptr) 
        {
            std::cerr << "Failed to allocate data buffer for index " << i << std::endl;
            free(buffer_return_[i]);
            // Clean up previously allocated buffers
            for (int j = 0; j < i; j++) 
            {
                if (buffer_return_[j] != nullptr) 
                {
                    if (buffer_return_[j]->data != nullptr) 
                    {
                        free(buffer_return_[j]->data);
                    }
                    free(buffer_return_[j]);
                }
            }
            return M100EmptyResponse;
        }

        // Initialize other fields
        buffer_return_[i]->capacity = 256;
        buffer_return_[i]->size = 0;
        buffer_return_[i]->head = 0;
        buffer_return_[i]->tail = 0;
        buffer_return_[i]->closed = false;
        buffer_return_[i]->loaded = false;
        // std::cout << "Completed allocation for buffer[" << i << "]" << std::endl;
    }
    std::cout<<"Return Buffer allocation complete"<<std::endl;
    assert((Buffer*)&buffer_return_[0] != nullptr);
    assert(buffer_return_ != nullptr);
    for(int i=0;i<5;i++)
    {
        assert(buffer_return_[i]->data != nullptr);
    }
    module->serial->send_wait_multi(buffer_return_,cmd, cmd_length); // (Buffer*)&buffer_return_[0]
    std::cout<<"Response captured"<<std::endl;
  
    for(int load=0;load<5;load++)
    {	
    	if(buffer_return_[load]->loaded)
    	{
            // if (module->serial->multi_buffer_[load].vec_data == nullptr) 
            // {
            //     module->serial->multi_buffer_[load].vec_data.resize(buffer_return_[load]->size);
            // }
            
            if (buffer_return_[load] == nullptr) 
            {
                std::cerr << "Error: buffer_return_[" << load << "] is null" << std::endl;
                continue;
            }

            if (!buffer_return_[load]->loaded) 
            {
                std::cerr << "Buffer " << load << " not loaded" << std::endl;
                continue;
            }

            if (buffer_return_[load]->data == nullptr) 
            {
                std::cerr << "Error: buffer_return_[" << load << "]->data is null" << std::endl;
                continue;
            }

            if (buffer_return_[load]->size == 0) 
            {
                std::cerr << "Warning: buffer_return_[" << load << "] has zero size" << std::endl;
                continue;
            }
            std::vector<uint8_t> test_epc;

            // assert(module->serial->multi_buffer_[load].vec_data != nullptr);
            // std::cout<<"Assertion complete for multi_buffer_["<<load<<"]"<<std::endl;
            assert(buffer_return_[load]->data != nullptr);
            // std::cout<<"Assertion complete for buffer_return_["<<load<<"]->data"<<std::endl;
            assert(buffer_return_[load] != nullptr);
            // std::cout<<"Assertion complete for buffer_return_["<<load<<"]"<<std::endl;

            int size = buffer_return_[load]->size;
            test_epc.resize(buffer_return_[load]->size);
            module->serial->multi_buffer_[load].vec_data.resize(buffer_return_[load]->size);

            std::copy(buffer_return_[load]->data,buffer_return_[load]->data + size, test_epc.begin());
            for(size_t i = 0; i < size; i++) 
            {
                std::cout << std::hex                    // Set hex format
                        << std::uppercase              // Use uppercase letters
                        << std::setw(2)               // Width of 2 characters
                        << std::setfill('0')          // Fill with zeros
                        << static_cast<int>(test_epc[i])  // Cast to int for proper display
                        << " ";                                                                                                                                                                                                   // Space between bytes
            }
            std::cout << std::dec << std::endl;

            std::cout<<"buffer_return_["<<load<<"] size: "<<size<<std::endl;
            std::copy(test_epc.begin(),test_epc.end(),module->serial->multi_buffer_[load].vec_data.begin());	
            // std::memcpy(module->serial->multi_buffer_[load].data, buffer_return_[load]->data, buffer_return_[load]->size);
            // std::cout<<"Copy complete for buffer["<<load<<"]"<<std::endl;

            module->serial->multi_buffer_[load].size = buffer_return_[load]->size;
            module->serial->multi_buffer_[load].loaded = true;
            std::cout<<"Loading complete for multi_buffer_["<<load<<"]"<<std::endl;	
    	}
    
    }
    
    std::cout<<"Response Loaded to Buffer Successfully\n"<<std::endl;
    // check if size > 0
     if(!buffer_return_[0]->loaded) 
     {
        printf("Serial Port Failed/Empty Response in setup_and_send_rx\n");
        for (int i = 0; i < 5; i++) 
        {
            if (buffer_return_[i] != nullptr) 
            {
                // if (buffer_return_[i]->data != nullptr) 
                // {
                //     free(buffer_return_[i]->data);
                // }
                free(buffer_return_[i]);
            }
        }
         return M100EmptyResponse;
     }
    int success = 0;
    for(int check = 0;check < 5 ;check++)
    {
        if(buffer_return_[check]->loaded)
        {
            // std::cout<<"multi_buffer_["<<check<<"]->loaded is TRUE"<<std::endl;
            std::cout<<"Starting validation for multi_buffer_["<<check<<"]"<<std::endl;
            uint8_t* data;
            std::copy(module->serial->multi_buffer_[check].vec_data.begin(),module->serial->multi_buffer_[check].vec_data.end(),data);
            // std::cout<<"Copying complete to temporary data buffer for multi_buffer_["<<check<<"]"<<std::endl;
            size_t length_ = module->serial->multi_buffer_[check].size;
            // check if data is valid
            if(data[0] != FRAME_START || data[length_ - 1] != FRAME_END || length_ < 24)
            {
                std::cout<<"Validation Failed for multi_buffer_["<<check<<"] "<<std::endl;
                module->serial->multi_buffer_[check].loaded = false;
            }
            // check if checksum is correct
            if(checksum(data + 1, length_ - 3) != data[length_ - 2])
            {
                std::cout<<"Checksum Failed for multi_buffer_["<<check<<"] "<<std::endl;
                module->serial->multi_buffer_[check].loaded = false;
            }
            if(module->serial->multi_buffer_[check].loaded)
            {
                success+=1;
            }
        }
    }
    
    // Clean up allocated memory
    for (int i = 0; i < 5; i++) 
    {
        if (buffer_return_[i] != nullptr) 
        {
            // if (buffer_return_[i]->data != nullptr) 
            // {
            //     free(buffer_return_[i]->data);
            // }
            free(buffer_return_[i]);
        }
    }
    // std::cout<<"setup and send rx multiple complete"<<std::endl;
    return success > 0 ? M100SuccessResponse : M100ValidationFail;

}

M100ModuleInfo* m100_module_info_alloc() {
    M100ModuleInfo* module_info = (M100ModuleInfo*)malloc(sizeof(M100ModuleInfo));
    return module_info;
}

void m100_module_info_free(M100ModuleInfo* module_info) {
    if(module_info->hw_version != NULL) free(module_info->hw_version);
    if(module_info->sw_version != NULL) free(module_info->sw_version);
    if(module_info->manufacturer != NULL) free(module_info->manufacturer);
    free(module_info);
}

M100Module* m100_module_alloc(UHFSerial* serial) {
    M100Module* module = (M100Module*)malloc(sizeof(M100Module));
    module->transmitting_power = DEFAULT_TRANSMITTING_POWER;
    module->region = DEFAULT_WORKING_REGION;
    module->info = m100_module_info_alloc();
    module->serial = serial;
    module->write_mask = WRITE_EPC; // default to write epc only
    return module;
}

void m100_module_free(M100Module* module) {
    // std::cout<<"Starting module free"<<std::endl;
    // m100_module_info_free(module->info);
    // std::cout<<"module->info free success"<<std::endl;
    // module->serial->stop();
    // std::cout<<"module->serial->stop() success"<<std::endl;
    // uhf_serial_free(module->serial);
    free(module);
    // std::cout<<"free(module) success"<<std::endl;
}

uint8_t checksum(const uint8_t* data, size_t length) {
    // CheckSum8 Modulo 256
    // Sum of Bytes % 256
    uint64_t sum_val = 0x00;
    for(size_t i = 0; i < length; i++) {
        sum_val += data[i];
    }
    return (uint8_t)(sum_val % 0x100);
}

uint16_t crc16_genibus(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF; // Initial value
    uint16_t polynomial = 0x1021; // CRC-16/GENIBUS polynomial

    for(size_t i = 0; i < length; i++) {
        crc ^= (data[i] << 8); // Move byte into MSB of 16bit CRC
        for(int j = 0; j < 8; j++) {
            if(crc & 0x8000) {
                crc = (crc << 1) ^ polynomial;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc ^ 0xFFFF; // Post-inversion
}

static char* _m100_info_helper(M100Module* module, char** info_field) {
    if (!module || !module->info || !info_field) {
        return NULL;
    }
    return *info_field;
}
// char* _m100_info_helper(M100Module* module, char** info) {
//     if(!uhf_buffer_get_size(module->serial->buffer)) return NULL;
//     uint8_t* data = uhf_buffer_get_data(module->serial->buffer);
//     uint16_t payload_len = data[3];
//     payload_len = (payload_len << 8) + data[4];
//     FuriString* temp_str = furi_string_alloc();
//     for(int i = 0; i < payload_len; i++) {
//         furi_string_cat_printf(temp_str, "%c", data[6 + i]);
//     }
//     if(*info == NULL) {
//         *info = (char*)malloc(sizeof(char) * payload_len);
//     } else {
//         for(size_t i = 0; i < strlen(*info); i++) {
//             (*info)[i] = 0;
//         }
//     }
//     memcpy(*info, furi_string_get_cstr(temp_str), payload_len);
//     furi_string_free(temp_str);
//     return *info;
// }

char* m100_get_hardware_version(M100Module* module) {
    setup_and_send_rx(module, (uint8_t*)&CMD_HW_VERSION.cmd[0], CMD_HW_VERSION.length);
    return _m100_info_helper(module, &module->info->hw_version);
}
char* m100_get_software_version(M100Module* module) {
    setup_and_send_rx(module, (uint8_t*)&CMD_SW_VERSION.cmd[0], CMD_SW_VERSION.length);
    return _m100_info_helper(module, &module->info->sw_version);
}
char* m100_get_manufacturers(M100Module* module) {
    setup_and_send_rx(module, (uint8_t*)&CMD_MANUFACTURERS.cmd[0], CMD_MANUFACTURERS.length);
    return _m100_info_helper(module, &module->info->manufacturer);
}

M100ResponseType m100_single_poll(M100Module* module, UHFTag* uhf_tag) {
    // printf("Initializing Single Poll\n");
    M100ResponseType rp_type =
        setup_and_send_rx(module, (uint8_t*)&CMD_SINGLE_POLLING.cmd[0], CMD_SINGLE_POLLING.length);
    if(rp_type != M100SuccessResponse) 
    {
        //  printf("setup_and_send_rx failed\n");
        return rp_type;
    }
    uint8_t* data = module->serial->buffer_->data;
    uint16_t pc = data[6];
    uint16_t crc = 0;
    // mask out epc length from protocol control
    size_t epc_len = pc;
    epc_len >>= 3;
    epc_len *= 2;
    // get protocol control
    pc <<= 8;
    pc += data[7];
    // get cyclic redundency check
    crc = data[8 + epc_len];
    crc <<= 8;
    crc += data[8 + epc_len + 1];
    // validate crc
    if(crc16_genibus(data + 6, epc_len + 2) != crc) return M100ValidationFail;
    uhf_tag_set_epc_pc(uhf_tag, pc);
    uhf_tag_set_epc_crc(uhf_tag, crc);
    uhf_tag_set_epc(uhf_tag, data + 8, epc_len);
    // std::cout<<"M100 Single Poll function completed"<<std::endl;
    return M100SuccessResponse;
}

M100ResponseType m100_multi_poll(M100Module* module) 
{
    printf("Initializing Multi Poll\n");
    M100ResponseType rp_type =
        setup_and_send_rx_multiple(module, (uint8_t*)&CMD_MULTIPLE_POLLING.cmd[0], CMD_MULTIPLE_POLLING.length);
    if(rp_type != M100SuccessResponse) 
    {
         printf("setup_and_send_rx failed\n");
        return rp_type;
    }
    std::cout<<"Completed setup and send rx multiple"<<std::endl;
    module->tag_no = 0;
    for(int check=0;check<5;check++)
    {
        if(module->serial->multi_buffer_[check].loaded)
        {
            module->tag_no += 1;
            std::cout<<"multi_buffer_["<<check<<"]->loaded is TRUE"<<std::endl;
        }
    }
    std::cout<<"m100 multi poll complete"<<std::endl;
    return M100SuccessResponse;
}

M100ResponseType multi_poll_tag_alloc(M100Module* module, UHFTag** uhf_tag)
{   
    uint16_t crc_buffer[5];
    bool load_tag = true;
    for(int check=0,tag=0;check<5;check++)
    {   
        if(module->serial->multi_buffer_[check].loaded)
        {
            // uint8_t* data;
            std::vector<uint8_t> data;
            data.resize(module->serial->multi_buffer_[check].vec_data.size());
            std::cout<<"Starting Copy"<<std::endl;
            std::copy(module->serial->multi_buffer_[check].vec_data.begin(),module->serial->multi_buffer_[check].vec_data.end(),data.begin());
            std::cout<<"Copying complete from multi_buffer_["<<check<<"] to temporary data buffer"<<std::endl;
            uint16_t pc = static_cast<uint16_t>(data[6]);
            uint16_t crc = 0;

            // mask out epc length from protocol control
            size_t epc_len = pc;
            epc_len >>= 3;
            epc_len *= 2;

            // get protocol control
            pc <<= 8;
            pc += static_cast<uint16_t>(data[7]);

            // get cyclic redundency check
            crc = static_cast<uint16_t>(data[8 + epc_len]);
            crc <<= 8;
            crc += static_cast<uint16_t>(data[8 + epc_len + 1]);

            // validate crc
            if(crc16_genibus(data.data() + 6, epc_len + 2) != crc) return M100ValidationFail;
            uhf_tag[tag]->epc->pc = pc;
            uhf_tag[tag]->epc->crc = crc;
            std::cout<<"Starting memcpy from temp data buffer to uhf_tag["<<tag<<"]->epc->data"<<std::endl;
            std::memcpy(uhf_tag[tag]->epc->data,data.data() + 8,epc_len);
            std::cout<<"memcpy completed successfully"<<std::endl;
            uhf_tag[tag]->epc->size = epc_len; 
            tag++; 
        }
        else
        {
            std::cout<<"multi_buffer_["<<check<<"]->loaded is False"<<std::endl;
        }
    }

    return M100SuccessResponse;
}

M100ResponseType m100_stop_multi_poll(M100Module* module)
{
    std::cout<<"Starting m100_stop_multi_poll"<<std::endl;
    module->serial->buffer_->multi = true;
    M100ResponseType rp_type =
        setup_and_send_rx(module, (uint8_t*)&CMD_STOP_MULTIPLE_POLLING.cmd[0], CMD_STOP_MULTIPLE_POLLING.length);
    if(rp_type != M100SuccessResponse) 
    {
        std::cout<<"setup_and_send_rx failed"<<std::endl;
        return rp_type;
    }

    std::cout<<"STOP SUCCESSFULL"<<std::endl;
    // uint8_t* data = module->serial->buffer_->data;
    // size_t length = sizeof(data);
    // printf("stop multi_poll response: ");
    // for(size_t i = 0; i < length; i++) {
    //     printf("%02X ", data[i]);
    // }
    return M100SuccessResponse;
}

M100ResponseType m100_set_select(M100Module* module, UHFTag* uhf_tag) {
    // Set select
    uint8_t cmd[MAX_BUFFER_SIZE];
    size_t cmd_length = CMD_SET_SELECT_PARAMETER.length;
    size_t mask_length_bytes = uhf_tag->epc->size;
    size_t mask_length_bits = mask_length_bytes * 8;
    // payload len == sel param len + ptr len + mask len + epc len
    size_t payload_len = 7 + mask_length_bytes;
    memcpy(cmd, CMD_SET_SELECT_PARAMETER.cmd, cmd_length);
    std::cout<<"memcpy 1 in select done...\n";
    // set new length
    cmd_length = 12 + mask_length_bytes + 2;
    // set payload length
    cmd[3] = (payload_len >> 8) & 0xFF;
    cmd[4] = payload_len & 0xFF;
    // set select param
    cmd[5] = 0x01; // 0x00=rfu, 0x01=epc, 0x10=tid, 0x11=user
    // set ptr
    cmd[9] = 0x20; // epc data begins after 0x20
    // set mask length
    cmd[10] = mask_length_bits;
    // truncate
    cmd[11] = false;
    // set mask
    memcpy((void*)&cmd[12], uhf_tag->epc->data, mask_length_bytes);
    std::cout<<"memcpy 2 in select done...\n";
    // set checksum
    cmd[cmd_length - 2] = checksum(cmd + 1, 11 + mask_length_bytes);
    // end frame
    cmd[cmd_length - 1] = FRAME_END;

    M100ResponseType rp_type = setup_and_send_rx(module, cmd, 12 + mask_length_bytes + 3);

    if(rp_type != M100SuccessResponse) return rp_type;

    uint8_t* data = module->serial->buffer_->data;
    if(data[5] != 0x00) return M100ValidationFail; // error if not 0

    return M100SuccessResponse;
}

void m100_enable_write_mask(M100Module* module, WriteMask mask) {
    module->write_mask |= mask;
}

void m100_disable_write_mask(M100Module* module, WriteMask mask) {
    module->write_mask &= ~mask;
}

bool m100_is_write_mask_enabled(M100Module* module, WriteMask mask) {
    return (module->write_mask & mask) == mask;
}

UHFTag* m100_get_select_param(M100Module* module) {
    uhf_buffer_reset(module->serial->buffer_);
    // furi_hal_uart_set_irq_cb(FuriHalUartIdLPUART1, rx_callback, module->serial->buffer);
    // furi_hal_uart_tx(
    //     FuriHalUartIdUSART1,
    //     (uint8_t*)&CMD_GET_SELECT_PARAMETER.cmd,
    //     CMD_GET_SELECT_PARAMETER.length);
    // furi_delay_ms(DELAY_MS);
    // UHFTag* uhf_tag = uhf_tag_alloc();
    // uint8_t* data = buffer_get_data(module->serial->buffer);
    // size_t mask_length =
    // uhf_tag_set_epc(uhf_tag, data + 12, )
    // TODO : implement
    return NULL;
}

M100ResponseType m100_read_label_data_storage(
    M100Module* module,
    UHFTag* uhf_tag,
    BankType bank,
    uint32_t access_pwd,
    uint16_t word_count) {
    std::cout<<"In read label data.....\n";
    /*
        Will probably remove UHFTag as param and get it from get selected tag
    */
    if(bank == EPCBank) return M100SuccessResponse;
    uint8_t cmd[MAX_BUFFER_SIZE];
    size_t cmd_length = CMD_READ_LABEL_DATA_STORAGE_AREA.length;
    memcpy(cmd, CMD_READ_LABEL_DATA_STORAGE_AREA.cmd, cmd_length);
    // set access password
    cmd[5] = (access_pwd >> 24) & 0xFF;
    cmd[6] = (access_pwd >> 16) & 0xFF;
    cmd[7] = (access_pwd >> 8) & 0xFF;
    cmd[8] = access_pwd & 0xFF;
    // set mem bank
    cmd[9] = (uint8_t)bank;
    // set word counter
    cmd[12] = (word_count >> 8) & 0xFF;
    cmd[13] = word_count & 0xFF;
    // calc checksum
    cmd[cmd_length - 2] = checksum(cmd + 1, cmd_length - 3);

    M100ResponseType rp_type = setup_and_send_rx(module, cmd, cmd_length);
    if(rp_type != M100SuccessResponse) return rp_type;

    uint8_t* data = uhf_buffer_get_data(module->serial->buffer_);

    uint8_t rtn_command = data[2];
    uint16_t payload_len = data[3];
    payload_len = (payload_len << 8) + data[4];

    if(rtn_command == 0xFF) {
        if(payload_len == 0x01) 
        {
            return M100NoTagResponse;
        }
        else if(payload_len == 0xA3)
        {
            return M100MemoryOverrun;
        }
        else if(payload_len == 0x16)
        {
            return M100AccessPasswordWrong;
        }
    }

    size_t ptr_offset = 5 /*<-ptr offset*/ + uhf_tag_get_epc_size(uhf_tag) + 3 /*<-pc + ul*/;
    size_t bank_data_length = payload_len - (ptr_offset - 5 /*dont include the offset*/);
    std::cout<<"Read data copy...\n";
    std::vector<uint8_t> read_data;
    read_data.resize(bank_data_length);
    std::copy(data + ptr_offset, data + ptr_offset + bank_data_length, read_data.begin());

    if(bank == TIDBank) {
        uhf_tag_set_tid(uhf_tag, data + ptr_offset, bank_data_length);
    } 
    else if(bank == UserBank) {
        // uhf_tag_set_user(uhf_tag, data + ptr_offset, bank_data_length);
        uhf_tag_set_user(uhf_tag, read_data, bank_data_length);
    }
    std::cout<<"Received Tag Data size: "<<bank_data_length<<std::endl;

    return M100SuccessResponse;
}

M100ResponseType m100_write_label_data_storage(
    M100Module* module,
    UHFTag* saved_tag,
    UHFTag* selected_tag,
    BankType bank,
    uint16_t source_address,
    uint32_t access_pwd) 
{
    uint8_t cmd[MAX_BUFFER_SIZE];
    size_t cmd_length = CMD_WRITE_LABEL_DATA_STORE.length;
    memcpy(cmd, CMD_WRITE_LABEL_DATA_STORE.cmd, cmd_length);
    uint16_t payload_len = 9;
    uint16_t data_length = 0;
    if(bank == ReservedBank) {
        // access pwd len + kill pwd len
        payload_len += 4;
        data_length = 4;
    } else if(bank == EPCBank) {
        // epc len + pc len
        payload_len += 4 + uhf_tag_get_epc_size(saved_tag);
        data_length = 4 + uhf_tag_get_epc_size(saved_tag);
        // set data
        uint8_t tmp_arr[4];
        tmp_arr[0] = (uint8_t)((uhf_tag_get_epc_crc(selected_tag) >> 8) & 0xFF);
        tmp_arr[1] = (uint8_t)(uhf_tag_get_epc_crc(selected_tag) & 0xFF);
        tmp_arr[2] = (uint8_t)((uhf_tag_get_epc_pc(saved_tag) >> 8) & 0xFF);
        tmp_arr[3] = (uint8_t)(uhf_tag_get_epc_pc(saved_tag) & 0xFF);
        memcpy(cmd + 14, tmp_arr, 4);
        memcpy(cmd + 18, uhf_tag_get_epc(saved_tag), uhf_tag_get_epc_size(saved_tag));
    } else if(bank == UserBank) {
        payload_len += uhf_tag_get_user_size(saved_tag);
        data_length = uhf_tag_get_user_size(saved_tag);
        // set data
        std::cout<<"Starting userbank_data init .....\n";
        std::vector<uint8_t> userbank_data;
        userbank_data.resize(uhf_tag_get_user_size(saved_tag));
        userbank_data = uhf_tag_get_user(saved_tag);
        std::cout<<"Starting std::copy in m100_write UserBank .....\n";
        // memcpy(cmd + 14, uhf_tag_get_user(saved_tag), uhf_tag_get_user_size(saved_tag));
        std::copy(userbank_data.begin(),userbank_data.end(), cmd + 14);
        std::cout<<"Copy complete .....\n";
    }
    // set payload length
    cmd[3] = (payload_len >> 8) & 0xFF;
    cmd[4] = payload_len & 0xFF;
    // set access password
    cmd[5] = (access_pwd >> 24) & 0xFF;
    cmd[6] = (access_pwd >> 16) & 0xFF;
    cmd[7] = (access_pwd >> 8) & 0xFF;
    cmd[8] = access_pwd & 0xFF;
    // set membank
    cmd[9] = (uint8_t)bank;
    // set source address
    cmd[10] = (source_address >> 8) & 0xFF;
    cmd[11] = source_address & 0xFF;
    // set data length
    size_t data_length_words = data_length / 2;
    cmd[12] = (data_length_words >> 8) & 0xFF;
    cmd[13] = data_length_words & 0xFF;
    // update cmd len
    cmd_length = 7 + payload_len;
    // calculate checksum
    cmd[cmd_length - 2] = checksum(cmd + 1, cmd_length - 3);
    cmd[cmd_length - 1] = FRAME_END;
    // send cmd
    M100ResponseType rp_type = setup_and_send_rx(module, cmd, cmd_length);
    if(rp_type != M100SuccessResponse) return rp_type;
    uint8_t* buff_data = uhf_buffer_get_data(module->serial->buffer_);
    size_t buff_length = uhf_buffer_get_size(module->serial->buffer_);
    if(buff_data[2] == 0xFF && buff_length == 8)
        return M100NoTagResponse;
    else if(buff_data[2] == 0xFF)
        return M100ValidationFail;
    return M100SuccessResponse;
}
void m100_set_baudrate(M100Module* module, uint32_t baudrate) {
    size_t length = CMD_SET_COMMUNICATION_BAUD_RATE.length;
    uint8_t cmd[length];
    uint8_t* response_;
    memcpy(cmd, CMD_SET_COMMUNICATION_BAUD_RATE.cmd, length);
    uint16_t br_mod = baudrate / 100; // module format
    cmd[6] = 0xFF & br_mod; // pow LSB
    cmd[5] = 0xFF & (br_mod >> 8); // pow MSB
    cmd[length - 2] = checksum(cmd + 1, length - 3);
    // setup_and_send_rx(module, cmd, length);
    module->serial->send_wait(cmd, length,response_);
    module->serial->set_baudrate(baudrate);
    module->serial->baudrate_ = baudrate;
}

bool m100_set_working_region(M100Module* module, WorkingRegion region) {
    size_t length = CMD_SET_WORK_AREA.length;
    uint8_t cmd[length];
    memcpy(cmd, CMD_SET_WORK_AREA.cmd, length);
    cmd[5] = (uint8_t)region;
    cmd[length - 2] = checksum(cmd + 1, length - 3);
    setup_and_send_rx(module, cmd, length);
    module->region = region;
    return true;
}

bool m100_set_transmitting_power(M100Module* module, uint16_t power) {
    size_t length = CMD_SET_TRANSMITTING_POWER.length;
    uint8_t cmd[length];
    memcpy(cmd, CMD_SET_TRANSMITTING_POWER.cmd, length);
    cmd[5] = (power >> 8) & 0xFF;
    cmd[6] = power & 0xFF;
    cmd[length - 2] = checksum(cmd + 1, length - 3);
    setup_and_send_rx(module, cmd, length);
    module->transmitting_power = power;
    return true;
}

bool m100_set_freq_hopping(M100Module* module, bool hopping) {
    UNUSED(module);
    UNUSED(hopping);
    return true;
}

bool m100_set_power(M100Module* module, uint8_t* power) {
    UNUSED(module);
    UNUSED(power);
    return true;
}

uint32_t m100_get_baudrate(M100Module* module) {
    return module->serial->baudrate_;
}