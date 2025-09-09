// 장애물 리스트 융합 및 이전 데이터와 비교
std::vector<ObstacleData> mergeAndCompareLists(
    const std::vector<ObstacleData> &previousFusionList,
    std::vector<ObstacleData> listMain,
    std::vector<ObstacleData> listSub1,
    std::vector<ObstacleData> listSub2,
    const VehicleData &mainVehicle,
    const VehicleData &sub1Vehicle,
    const VehicleData &sub2Vehicle)
{
    std::vector<VehicleData> nonEmptyVehicles;
    std::vector<std::vector<ObstacleData>> nonEmptyLists;
    std::vector<ObstacleData> mergedList;

    // (250909) 특장차 후순위 적용
    if (worksub1)
    {
        nonEmptyVehicles.push_back(sub1Vehicle);
        nonEmptyLists.push_back(listSub1);
    }
    if (worksub2)
    {
        nonEmptyVehicles.push_back(sub2Vehicle);
        nonEmptyLists.push_back(listSub2);
    }
    if (workego)
    {
        nonEmptyVehicles.push_back(mainVehicle);
        nonEmptyLists.push_back(listMain);
    }

    // adcm::Log::Info() << "융합: 빈 데이터 제외 완료: " << nonEmptyLists.size() << ", " << nonEmptyVehicles.size();
    // 융합할 리스트 필터링
    if (nonEmptyLists.size() == 1)
    {
        // 유일한 리스트 하나가 있을 경우 그대로 사용
        mergedList = nonEmptyLists[0];
    }

    else
    {
        // 둘 이상 리스트가 있을 때 융합 수행
        auto handleFusionForPair = [&](const std::vector<ObstacleData> &listA, const std::vector<ObstacleData> &listB)
        {
            std::vector<ObstacleData> fusionList = listA;
            if (!listA.empty() && !listB.empty())
            {
                auto distMatrix = createDistanceMatrix(listB, listA);
                auto assignment = solveAssignment(distMatrix);
                // for (int i = 0; i < assignment.size(); i++)
                //     adcm::Log::Info() << "assignment" << i << ": " << assignment[i];
                processFusion(fusionList, listB, assignment);
            }
            return fusionList;
        };
        // 처음 두 개 리스트 융합
        // adcm::Log::Info() << "[first list]";
        // for (auto first : nonEmptyLists[0])
        //     adcm::Log::Info() << first.obstacle_class << ": [" << first.fused_position_x << ", " << first.fused_position_y << "]";
        // adcm::Log::Info() << "[second list]";
        // for (auto second : nonEmptyLists[1])
        //     adcm::Log::Info() << second.obstacle_class << ": [" << second.fused_position_x << ", " << second.fused_position_y << "]";
        mergedList = handleFusionForPair(nonEmptyLists[0], nonEmptyLists[1]);
        // adcm::Log::Info() << "융합: 융합 1번 완료";

        // 세 번째 리스트가 있다면 그 결과와 함께 융합
        if (nonEmptyLists.size() > 2)
        {
            // adcm::Log::Info() << "[third list]";
            // for (auto third : nonEmptyLists[2])
            //     adcm::Log::Info() << third.obstacle_class << ": [" << third.fused_position_x << ", " << third.fused_position_y << "]";
            mergedList = handleFusionForPair(mergedList, nonEmptyLists[2]);
            // adcm::Log::Info() << "융합: 융합 2번 완료";
        }
    }

    if (mergedList.empty())
    {
        if (previousFusionList.empty())
            adcm::Log::Info() << "장애물 리스트 비어있음";
        else
            adcm::Log::Info() << "현재 TimeStamp 장애물 X, 이전 TimeStamp 장애물리스트 그대로 사용";
        return previousFusionList;
    }
    else
    {
        if (previousFusionList.empty())
        {
            for (auto &obstacle : mergedList)
            {
                auto newId = id_manager.allocID();
                obstacle.obstacle_id = newId;
                adcm::Log::Info() << "새로운 장애물 " << obstacle.obstacle_class << " ID 할당: " << newId << " : [" << obstacle.fused_position_x << ", " << obstacle.fused_position_y << "]";
            }
            adcm::Log::Info() << "새로운 장애물 리스트 생성: " << id_manager.getNum();
            return mergedList;
        }

        else
        {
            adcm::Log::Info() << "previousFusionList size: " << previousFusionList.size();
            adcm::Log::Info() << "mergedList size: " << mergedList.size();
            for (auto merge : mergedList)
            {
                adcm::Log::Info() << merge.obstacle_id << "(" << merge.obstacle_class << ")"
                                  << ": [" << merge.fused_position_x << ", " << merge.fused_position_y << ", "
                                  << merge.fused_velocity_x << ", " << merge.fused_velocity_y << "]";
            }

            // adcm::Log::Info() << "융합: 이전 데이터와 융합하여 ID부여 시도";
            auto distMatrix = createDistanceMatrix(previousFusionList, mergedList);
            // adcm::Log::Info() << "융합: 거리배열 생성";
            auto assignment = solveAssignment(distMatrix);
            // adcm::Log::Info() << "융합: Munkres Algorithm 적용: " << assignment.size();
            // for (int i = 0; i < assignment.size(); i++)
            //     adcm::Log::Info() << "assignment" << i << ": " << assignment[i];
            processFusion(mergedList, previousFusionList, assignment);
            adcm::Log::Info() << "융합: 이전 데이터와 융합 완료";
            for (auto merge : mergedList)
            {
                adcm::Log::Info() << merge.obstacle_id << "(" << merge.obstacle_class << ")"
                                  << ": [" << merge.fused_position_x << ", " << merge.fused_position_y << ", "
                                  << merge.fused_velocity_x << ", " << merge.fused_velocity_y << "]";
            }
            // for (auto merge : mergedList)
            // {
            //     adcm::Log::Info() << "융합리스트 장애물id: " << merge.obstacle_id;
            // }
            // std::vector<ObstacleData> finalList;
            // assignIDsForNewData(finalList, mergedList, assignment);
            // adcm::Log::Info() << "융합: ID부여 완료: " << id_manager.getNum();
            return mergedList;
        }
    }
}

/////////// road_index 반영 코드 ////////////
// road_z -> road_list
std::vector<adcm::roadListStruct> ConvertRoadZToRoadList(const VehicleData &vehicle)
{
    std::vector<adcm::roadListStruct> result(24); // 0~22, 255
    for (int i = 0; i < 23; ++i)
    {
        result[i].road_index = i;
        result[i].Timestamp = vehicle.timestamp;
    }
    result[23].road_index = 255;
    result[23].Timestamp = vehicle.timestamp;

    double car_x = vehicle.position_x;
    double car_y = vehicle.position_y;
    double theta = vehicle.heading_angle * M_PI / 180.0;

    double cell_size = 0.1; // 10 cm 단위
    double start_x = 3.0;   // 전방 3m 제외
    double end_x = 10.0;    // 전방 최대 10m
    double box_width = 4.0; // 좌측 2m ~ 우측 2m

    size_t num_rows = 70;   // 전방 거리: (10m-3m)/0.1 = 70
    size_t num_cols = 40;   // 좌우 거리: 4m/0.1 = 40

    for (size_t i = 0; i < vehicle.road_z.size(); ++i)
    {
        uint8_t rz = vehicle.road_z[i];
        int idx_struct = (rz <= 22) ? rz : 23;

        size_t col = i / num_rows; // 0 ~ 39
        size_t row = i % num_rows; // 0 ~ 69

        // row : 0 -> 전방 10m, row : 69 -> 전방 3m
        double local_x = end_x - row * cell_size;
        double local_y = -box_width / 2 + col * cell_size;

        // 회전 변환
        double rotated_x = -local_x * sin(theta) - local_y * cos(theta);
        double rotated_y =  local_x * cos(theta) - local_y * sin(theta);

        double map_x = car_x + rotated_x;
        double map_y = car_y + rotated_y;

        adcm::map2dIndex idx = {map_x, map_y};
        result[idx_struct].map_2d_location.push_back(idx);
    }

    return result;
}

// 전송 전 맵 업데이트 코드 
void UpdateMapData(adcm::map_data_Objects &mapData, const std::vector<ObstacleData> &obstacle_list, const std::vector<VehicleData *> &vehicles)
{
    mapData.obstacle_list.clear();
    mapData.vehicle_list.clear();
    for (const auto &obstacle : obstacle_list)
        mapData.obstacle_list.push_back(ConvertToObstacleListStruct(obstacle, mapData.map_2d));

    for (const auto &vehicle : vehicles)
    {
        if (vehicle->vehicle_class != 0)
        {
            mapData.vehicle_list.push_back(ConvertToVehicleListStruct(*vehicle, mapData.map_2d));
            if (vehicle->road_z.size() > 1) // 시뮬레이션에선 road_z size가 1이므로 예외 처리
            {
                // 차량 road_z -> road_list 반환
                std::vector<adcm::roadListStruct> road_list = ConvertRoadZToRoadList(*vehicle);
                mapData.road_list.insert(mapData.road_list.end(), road_list.begin(), road_list.end());
            }
            else
                adcm::Log::Info() << vehicle->vehicle_class << " 차량 road_z 데이터 없음, road_list 반영 X";
        }
    }

    adcm::Log::Info() << "mapData 장애물 반영 완료 개수: " << mapData.obstacle_list.size();
    adcm::Log::Info() << "mapData 차량 반영 완료 개수: " << mapData.vehicle_list.size();
    adcm::Log::Info() << "mapData road_list 반영 완료: " << mapData.road_list.size();

    for (const auto &road : mapData.road_list)
    {
        adcm::Log::Info() << "road_index: " << static_cast<int>(road.road_index)
                          << ", count: " << road.map_2d_location.size();
    }

    map_2d_size = 0;

    return;
}