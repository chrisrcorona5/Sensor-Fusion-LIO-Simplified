#pragma once
#include "types.hpp"

class ImuReader {
public:
    explicit ImuReader(const std::string& drive_path);

    std::vector<ImuMeasurement> measurements() const;

private:
    std::string drive_path_;
    std::vector<double> loadTimestamps() const;
    std::vector<ImuMeasurement> parseOxts() const;
};