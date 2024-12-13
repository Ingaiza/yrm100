#pragma once
#ifndef UHF_MODULE_HPP
#define UHF_MODULE_HPP

#include <stdio.h>
#include <iostream>
#include <vector>
#include <optional>
#include "/home/aimbot/yrm100/src/yrm100/include/yrm100/uhf_module.hpp"
#include "/home/aimbot/yrm100/src/yrm100/include/yrm100/uhf_tag.hpp"



struct DEFAULT_PARAMS_
{
   uint32_t ACCESS_PSWD_ = 0x00000000;
   uint16_t WORD_COUNT_ = 0x0008;
   uint16_t SOURCE_ADDR_ = 0x0000;
};


uint8_t* single_poll();
// uint8_t* read(uint8_t* read_data);
uint8_t* read_select(uint8_t* read_data);
uint8_t* write_tag(uint8_t* write_data);
std::optional<std::vector<uint8_t>> multi_poll();

#endif