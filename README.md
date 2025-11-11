# edge_template
Edge DataFusion code template

## 2025.09.09 변경 사항
### mergeAndCompareLists() - 장애물 리스트 융합
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

특장차 장애물 리스트 후순위로 미룸

### ConvertRoadZToRoadList() - 차량 road_z -> road_list 반영

맵 ui 반영 순서
1) road_z -> road_list 벡터 형태로 데이터 저장
2) 별개로 adcm::map_data_Objects mapData 저장
3) road_list에서 index 0~22까지 map_2d_location 데이터 꺼내서 mapData에 반영 (누적)


## 2025.11.11 변경 사항
### ThreadReceiveWorkInfo() - WorkInfo 수신 후 맵 생성

