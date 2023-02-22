#include "ouster_main.h"

void sigint_handler(int signal) {
	if (keep_alive) {
		std::cout << "Received SIGIN. Stopping stream." << std::endl;
		keep_alive = false;
	} else {
		exit(signal);
	}
}


void FATAL(const char* msg) {
    std::cerr << msg << std::endl;
    std::exit(EXIT_FAILURE);
}


std::string get_current_time() {
	std::time_t current_time= time(nullptr);
	std::tm* local_time = std::localtime(&current_time);

	char time_string[25];
	std::strftime(time_string, sizeof(time_string), "%m%d_%H%M%S", local_time);

	return time_string;
}


void read_points(int argc, std::string scanner_ip) {
    sensor::init_logger("info", "ouster.log");

    const std::string sensor_hostname = scanner_ip;
    const std::string data_destination = "";
    std::cerr << "Connecting to \"" << sensor_hostname << "\"...\n";

    auto handle = sensor::init_client(sensor_hostname, data_destination);
    if (!handle) FATAL("Failed to connect");
    std::cerr << "Connection to sensor succeeded" << std::endl;
    
    auto metadata = sensor::get_metadata(*handle);
    sensor::sensor_info info = sensor::parse_metadata(metadata);
    size_t w = info.format.columns_per_frame;
    size_t h = info.format.pixels_per_column;
    ouster::sensor::ColumnWindow column_window = info.format.column_window;
    std::vector<LidarScan> scans{N_SCANS, LidarScan{w, h, info.format.udp_profile_lidar}};

    sensor::packet_format pf = sensor::get_format(info);
    XYZLut lut = ouster::make_xyz_lut(info);

    ScanBatcher batch_to_scan(info.format.columns_per_frame, pf);
    auto packet_buf = std::make_unique<uint8_t[]>(UDP_BUF_SIZE);

    uint i = 0u;
    std::vector<LidarScan::Points> clouds;

    std::array<float, 4> point;
    std::vector<std::array<float, 4>> pointcloud;

    while(true) {
        sensor::client_state st = sensor::poll_client(*handle);
        if (st & sensor::CLIENT_ERROR) FATAL("Sensor client returned error state!");

        if (st & sensor::LIDAR_DATA) {
            if (!sensor::read_lidar_packet(*handle, packet_buf.get(), pf)) {
                FATAL("Failed to read a packet of the expected size!");
            }

            if (batch_to_scan(packet_buf.get(), scans[i])) {
                if (scans[i].complete(info.format.column_window)) {
                    clouds.push_back(ouster::cartesian(scans[i], lut));
                    pointcloud.clear();
                    for(int j = 0; j < clouds[i].rows(); j++) {
                        auto xyz = clouds[i].row(j);
                        if (!xyz.isApproxToConstant(0.0)) {
                            point.at(0) = xyz(0);
                            point.at(1) = xyz(1);
                            point.at(2) = xyz(2);
                            point.at(3) = 0.0f;
                            pointcloud.push_back(point);
                        }
                    }
                    pcd.push(pointcloud);
                    i++;
                }
            }
        }
    }
}


void log_points() {
	std::string log_path = "/mnt/data/poc/";
	std::string log_name = log_path + get_current_time() + ".h5";
	H5::H5File log_file = H5::H5File(log_name, H5F_ACC_TRUNC);

	unsigned int group_idx = 0u;
	char idx_char[7];
	while(keep_alive) {
		if(!pcd.empty()) {
			sprintf(idx_char, "%06u", group_idx);
			std::string group_name = std::string(idx_char);
			std::cerr << group_name << std::endl;

			H5::Group group(log_file.createGroup(group_name));
			
			/* pcd */
			auto pcd_log = pcd.pop();
			hsize_t pcd_drow = pcd_log.size();
			hsize_t pcd_dcol = 4;
			hsize_t pcd_dims[2] = {pcd_drow, pcd_dcol};

			H5::DataSpace pcd_dspace = H5::DataSpace(2, pcd_dims);
			H5::DataSet pcd_dset = H5::DataSet(group.createDataSet("pointcloud", H5::PredType::NATIVE_FLOAT, pcd_dspace));
			pcd_dset.write(pcd_log.data(), H5::PredType::NATIVE_FLOAT);
			pcd_dset.close();
			pcd_dspace.close();

			group.close();
			group_idx++;
		}
	}

	log_file.close();
}


int main(int argc, char* argv[]) {
    std::signal(SIGINT, sigint_handler);
	keep_alive = true;

    std::string ip = "10.5.5.99";
	if(1 < argc) ip = argv[1];

	std::thread streaming(read_points, argc, ip);
    std::thread logging(log_points);
    
    streaming.join();
    logging.join();



    return EXIT_SUCCESS;
}
