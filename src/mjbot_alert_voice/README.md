# 개요
위험 감지 & 음성 기능 포팅 + 음성 기능 추가 개발

## Main 코드
- [위험 감지 & 음성 기능 통합 실행 코드](main.py)
- [위험 감지 단독 실행 코드](Alert/main.py)
- [음성 기능 단독 실행 코드](Chat/main.py)

## 체크리스트
### 포팅
- 모듈 설치 requirements.txt 작성 => (O) [requirements.txt 설치하러 가기](requirements.txt)    
- API KEY 교체 => (작업중) Googlemap api 교체 완료 / Twilio api 교체 필요 (우선 주석 처리함)  
- 파일 경로 교체 => (작업중) 음성 부정 발화 정보 파일 경로 교체 / 위험 감지 gps 와 mq-7 경로 확인 필요   
- json 파일 통일 => (작업중)
- 음성 기능 오렌지 보드 위에서 작동 완료, 포팅 성공 => (O)
- GPS , 화재 발생 함수화 시켜서 음성에 병합 => (O)

### 음성
- 먼저 말거는 기능 (개발 완료) 
- 이전에 했던 대화를 저장해서, 대화가 이어지게 하는 기능 (개발 완료)
- 부정적 발화 검출 기능을 좀 더 빠르게 작동되게 코드 리팩토링 => (개발 완료)
- 문답 속도가 느려서 고민중, 부정적 발화 기능을 주석처리하고 실행했음에도 느림 
- (학교 와이파이 - 7초) (집 와이파이 - 2초) => 녹음 시간을 실험을 위해 5초에서 3초로 줄임 
- 성별이랑 이름 넣는 기능 추가 => (개발 완료)
- 감정 확인 기능 성공 =>(개발 완료) 

### ROS
- 노드 생성 후, Publisher 와 Subscriber 개발 => (성공한듯..?)


==> 모듈 설치할 커맨드 추가

pip install ffmpeg-python
sudo apt-get install libportaudio2
pip install ffprobe-python
apt-get install ffmpeg
