#ifndef OUSTER_MAIN_H
#define OUSTER_MAIN_H

#include <stdio.h>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>
#include <unistd.h>
#include <thread>
#include <ctime>
#include <csignal>

#include "ouster/client.h"
#include "ouster/impl/build.h"
#include "ouster/lidar_scan.h"
#include "ouster/types.h"

#include <H5Cpp.h>
#include "RingBuffer.h"

using namespace ouster;

const size_t N_SCANS = 1000;
const size_t UDP_BUF_SIZE = 65536;
const size_t RING_BUF_SIZE = 10;

bool keep_alive;
RingBuffer<std::vector<std::array<float, 4>>> pcd(RING_BUF_SIZE);


#endif