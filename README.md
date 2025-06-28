# ROS2를 활용한 로봇 자동화 공정 시스템 구현 프로젝트


# 주제 : Auto Weld(용접협동로봇)
두산로봇팔 모델 사용<br> 
-ROBOT_ID = "dsr01" <br>
-ROBOT_MODEL = "m0609"

## 실제 시연 영상 : https://youtu.be/wykA4MYREYk

dsr_rokey/rokey/rokey/basic 안의 파일내용

## 작성하여 구현한 코드들
• 규칙적인 2D 용접 기능
• 불규칙적인 2D 용접 기능
• 규칙적인 3D 용접
• 안전기능

## 노드별 시나리오 설명
#### 규칙적인 2D 용접.py
1. UI노드와 제어노드로 분리되어 (규칙적인2D용접기능_UI.py), (규칙적인2D용접기능_구동.py) 2개의 파일로 구성

2. UI노드에서 직접 교시로 시작점과 도착점에 대한 좌표를 제어노드로 publish

![규칙적인 2D 용접 기능 UI](https://github.com/user-attachments/assets/e469747a-4e20-47d1-b025-00048b228b06)

3. 제어노드에서 시작점과 도착점에 대한 좌표를 전달받으면, 선형이동 시작


![규칙적인 2D 용접 py - 노드 통신 구조](https://github.com/user-attachments/assets/a66a08c7-2903-4365-af4d-7555b3660348)



#### 불규칙적인 2D 용접 기능.py


#### 규칙적인 3D 용접.py




![규칙적인 3D 용접 UI 사진](https://github.com/user-attachments/assets/64019536-f98f-492c-b807-309c86acbcbd)
#### 안전기능.py



