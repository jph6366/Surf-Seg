#include <memory>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include "loader/ASCIILoader.h"


class Timer {
    public:
        Timer() {
            m_StartTimePoint = std::chrono::high_resolution_clock::now();
        }
        ~Timer() {
            Stop();        
        }
        void Stop() {
            auto endTimePoint = std::chrono::high_resolution_clock::now();
            auto start = std::chrono::time_point_cast<std::chrono::microseconds>(m_StartTimePoint).time_since_epoch().count();
            auto end = std::chrono::time_point_cast<std::chrono::microseconds>(endTimePoint).time_since_epoch().count();        
            auto duration = end - start;
            double ms = duration * 0.001;
            std::cout << duration << "us (" << ms  << "ms)\n";    
        }
    private:
        std::chrono::time_point< std::chrono::high_resolution_clock> m_StartTimePoint;
};


int main(int argc, char *argv[]) {
    if(argv[1] == NULL)
    {
        throw std::runtime_error("Missing argument.");
    }
    renderVertices(argv[1]);
    return 0;
}
