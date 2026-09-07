#pragma once

struct VehicleInput
{
    // -1: 왼쪽
    //  0: 조향 입력 없음
    // +1: 오른쪽
    int steer = 0;

    // -1: 후진
    //  0: 구동 입력 없음
    // +1: 전진
    int drive = 0;

    // 차량 초기화 요청
    // 연속 상태가 아니라 한 번 발생하는 버튼 이벤트
    bool reset = false;
};