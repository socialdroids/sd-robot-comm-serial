#ifndef INCLUDE_INCLUDE_CONTROL_LOGGER_HPP_
#define INCLUDE_INCLUDE_CONTROL_LOGGER_HPP_

#include <array>
#include <cstddef>
#include <fstream>
#include <iomanip> // Required for std::setprecision, std::fixed
#include <iostream>

class ControlLogger
{
public:
    ControlLogger()
    {
        time.fill(0.f);
        input.fill(0.f);
        output.fill(0.f);

        samples = 0;
        dataSaved = false;
    }
    ~ControlLogger()
    {
    }

    void addData(uint64_t _time, float _input, float _output)
    {
        if (samples < MAX_SAMPLES)
        {
            // std::cout << std::fixed << std::setprecision(10);
            // std::cout << samples << ")" << _time << " " << _input << " "
            //           << _output << "\n";
            time[samples] = _time;
            input[samples] = _input;
            output[samples] = _output;
            samples++;
        }
    }

    bool isFull()
    {
        return samples >= MAX_SAMPLES;
    }

    bool isSaved()
    {
        return dataSaved;
    }

    bool saveFile()
    {
        if (dataSaved)
        {
            return true;
        }

        std::ofstream outputFile("control_data.csv");

        if (!outputFile.is_open())
        {
            std::cerr << "Error opening file!\n";
            return false;
        }
        outputFile << std::fixed << std::setprecision(10);
        outputFile << "T;U;Y\n";
        for (size_t n = 0; n < samples; n++)
        {
            outputFile << time.at(n) << "," << input.at(n) << ","
                       << output.at(n) << "\n";
        }

        outputFile.close();

        dataSaved = true;
        return dataSaved;
    }

private:
    static constexpr size_t MAX_SAMPLES = 100 * 60;
    std::array<float, MAX_SAMPLES> input, output;
    std::array<uint64_t, MAX_SAMPLES> time;
    size_t samples;
    bool dataSaved;
};

#endif // INCLUDE_INCLUDE_CONTROL_LOGGER_HPP_
