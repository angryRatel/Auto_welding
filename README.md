# ROS2를 활용한 로봇 자동화 공정 시스템 구현 프로젝트


# 주제 : Auto Weld(용접협동로봇)
두산로봇팔 모델 사용<br> 
-ROBOT_ID = "dsr01" <br>
-ROBOT_MODEL = "m0609"

## 실제 시연 영상 : https://youtu.be/wykA4MYREYk

### 코드 사용시 :  https://github.com/ROKEY-SPARK/DoosanBootcam3rdCo1.git 받아서 파일구조 수정후 사용할것 



#### dsr_rokey/rokey/rokey/basic 안의 파일내용

## 작성하여 구현한 코드들
• 규칙적인 2D 용접 기능
• 불규칙적인 2D 용접 기능
• 규칙적인 3D 용접
• 안전기능

## 노드별 시나리오 설명
#### 규칙적인 2D 용접.py
UI노드와 제어노드로 분리되어 (규칙적인2D용접기능_UI.py), (규칙적인2D용접기능_구동.py) 2개의 파일로 구성

1. UI노드에서 직접 교시로 시작점과 도착점에 대한 좌표를 제어노드로 publish

<img src="https://github.com/user-attachments/assets/e469747a-4e20-47d1-b025-00048b228b06" width="500" heigh = " 500"> <br> 

2. 제어노드에서 시작점과 도착점에 대한 좌표를 전달받으면, 선형이동 시작<br> 


![규칙적인 2D 용접 py - 노드 통신 구조](https://github.com/user-attachments/assets/a66a08c7-2903-4365-af4d-7555b3660348)


#### 불규칙적인 2D 용접 기능.py
용접할 부분의 도면이나, 센서 등으로 용접로봇이 이동할 경로 정보를 얻어올 수 있다는 시나리오를 가정
1. 경로 이미지 파일을 업로드하면, 일정한 간격으로 용접로봇이 이동할 포인트들을 추출

<img src="https://github.com/user-attachments/assets/7cf1485a-9ca2-4930-8a24-a68b1d7a28ff" width="500" heigh = " 500" ><br> 
2. 해당 포인트가 로봇의 작동범위의 최대 최소값을 벗어나지 않도록 스케일링<br> 
3. 스케일링된 좌표들을 csv 파일에 저장하여, 해당 좌표를 순차적으로 읽어 경로이동


![불규칙적인 2D 용접 기능 py - 노드구조](https://github.com/user-attachments/assets/52052adb-73c8-47b4-997b-3c75abac1c29)



#### 규칙적인 3D 용접.py
1. GUI 실행으로 실시간 로봇 좌표 확인
2. 용접 접합부 위치 직접교시를 위해GUI 실행과 동시에 수동모드 활성화
3. 캡처 버튼을 통해 좌표값 기록
4. 버튼을 통해 좌표값 CSV 저장 및 원 중심/ 반지름 계산, 홈 정렬 기능
5. 개별적 movec 노드를 통해 동작 (P3 -> P1 사이클 타임 단축)


<img src="https://github.com/user-attachments/assets/3616910a-82c0-49c0-b7d4-073c56b5a90b" width="500" heigh = " 500" > <br>

![규칙적인 3D 용접 py - 노드 통신 구조](https://github.com/user-attachments/assets/81421d74-08e1-4a6a-a9a2-2ec8a4b83d33)


###### ※두산로봇팔에는 DR_MV_ORI_RADIAL: 원주구속자세가 지원이 되지만 ros2에는 지원되지 않고 시간이 부족하여 2D상에서 세점을 잡아 그리는걸로 표현을 했고 추후 발전과제로 남겼다※


#### 안전기능.py
힘감지시 소리를 내는 코드를 추가적으로 규칙적인 2D에 넣었다

  #충격 감지 함수<br>
def monitor_impact(get_tool_force, get_current_posx, movel, VELOCITY, ACC):
이부분을 확인하면 된다


<img src="https://github.com/user-attachments/assets/bdf16ad0-dcb3-438e-bfac-0f345daa175d" width="500" heigh = " 500" >

###### 추가 내용 pdf 파일 확인

