# Xycar_ADproject
창업연계공학설계입문 05분반 01조

## 모범운전 Xycar

### 현실속 도로에서 발생할 수 있는 상황에 대처하는 알고리즘을 구성한다.


### reference : https://github.com/ghostbbbmt/Traffic-Sign-Detection


### 상황 설정

1. 노란색 감지
    : 저속운행

2. 초록색 감지
    : 정상 속도 운행

3. 빨간색 감지
    : 정지

4. 빨간색 사라짐
    : 재출발


### 소비자에게 보여줄 데이터

1. 속도 from autodrive1

2. 도로 교통 상황 from color dectector
    - 색
    - 색의 의미

3. IMU센서로부터 얻어드린 from IMU
   pitch, roll, yaw,
   차의 현재 상태를 알려주는데 도움이 될 듯 하다.

