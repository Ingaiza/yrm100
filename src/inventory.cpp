#include "/home/ingaiza/yrm_module/src/yrm100/include/yrm100/inventory.hpp"


// Helper functions for byte manipulation
void RFIDDataMemory::writeUint16(size_t offset, uint16_t value) {
    data[offset] = static_cast<uint8_t>((value >> 8) & 0xFF);
    data[offset + 1] = static_cast<uint8_t>(value & 0xFF);
}

uint16_t RFIDDataMemory::readUint16(size_t offset) const {
    return (static_cast<uint16_t>(data[offset]) << 8) | 
            static_cast<uint16_t>(data[offset + 1]);
}

RFIDDataMemory::RFIDDataMemory() : data(16, 0) {}

// Get the raw data
const std::vector<uint8_t>& RFIDDataMemory::getData() const {
    return data;
}

// Set date (days since Jan 1, 2000)
void RFIDDataMemory::setDate(const std::tm& date) {
    std::tm base = {};
    base.tm_year = 100;  // 2000 (year since 1900)
    base.tm_mday = 1;    // 1st day
    base.tm_mon = 0;     // January

    std::time_t baseTime = std::mktime(&base);
    std::time_t inputTime = std::mktime(const_cast<std::tm*>(&date));
    
    uint16_t days = static_cast<uint16_t>((inputTime - baseTime) / (60 * 60 * 24));
    writeUint16(0, days);
}

// Get stored date
std::tm RFIDDataMemory::getDate() const {
    uint16_t days = readUint16(0);
    std::tm base = {};
    base.tm_year = 100;  // 2000
    base.tm_mday = 1;
    base.tm_mon = 0;
    
    std::time_t baseTime = std::mktime(&base);
    std::time_t targetTime = baseTime + (static_cast<std::time_t>(days) * 24 * 60 * 60);
    
    std::tm* result = std::localtime(&targetTime);
    return *result;
}

void RFIDDataMemory::setProductCategory(uint16_t categoryId) {
    writeUint16(2, categoryId);
}

uint16_t RFIDDataMemory::getProductCategory() const {
    return readUint16(2);
}

void RFIDDataMemory::setLocation(uint16_t locationCode) {
    writeUint16(4, locationCode);
}

uint16_t RFIDDataMemory::getLocation() const {
    return readUint16(4);
}

void RFIDDataMemory::setStatus(bool inStock, bool reserved,bool damaged, bool expired) 
{
    uint8_t status = 0;
    if (inStock) status |= 0x01;
    if (reserved) status |= 0x02;
    if (damaged) status |= 0x04;
    if (expired) status |= 0x08;
    data[6] = status;
}

void RFIDDataMemory::setQuantity(uint8_t quantity) {
    data[7] = quantity;
}

uint8_t RFIDDataMemory::getQuantity() const {
    return data[7];
}

void RFIDDataMemory::setPrice(uint16_t priceInCents) {
    writeUint16(8, priceInCents);
}

uint16_t RFIDDataMemory::getPrice() const {
    return readUint16(8);
}

void RFIDDataMemory::setSupplierId(uint16_t supplierId) {
    writeUint16(10, supplierId);
}

uint16_t RFIDDataMemory::getSupplierId() const {
    return readUint16(10);
}

void RFIDDataMemory::setBatchNumber(uint16_t batchNum) {
    writeUint16(12, batchNum);
}

uint16_t RFIDDataMemory::getBatchNumber() const {
    return readUint16(12);
}

void RFIDDataMemory::calculateChecksum() {
    uint16_t sum = 0;
    for (size_t i = 0; i < 14; ++i) {
        sum += data[i];
    }
    writeUint16(14, sum);
}

bool RFIDDataMemory::verifyChecksum() const {
    uint16_t sum = 0;
    for (size_t i = 0; i < 14; ++i) {
        sum += data[i];
    }
    return sum == readUint16(14);
}