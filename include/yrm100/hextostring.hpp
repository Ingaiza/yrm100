#pragma once

#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <iostream>

class HexChunkProcessor {
private:
    static constexpr size_t CHUNK_SIZE = 12;

    //function to convert a single chunk of bytes to hex string

public:
    static std::string bytesToHexString(const std::vector<uint8_t>& bytes, size_t start, size_t length);
    // Process hex data in chunks
    static std::vector<std::string> processHexChunks(std::vector<uint8_t>& data);

    static std::vector<std::string> processHexChunksNonDestructive(const std::vector<uint8_t>& data);

};
