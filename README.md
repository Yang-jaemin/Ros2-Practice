# robot_programming practice

## Node1: topic, service server, action server
  - Topic Publisher: 학번을 발행
  - Service Server : 두 정수를 입력 받아 곱셈
  - Action Server: 학번를 받아 한 번에 하나의 숫자를 더하여 중간 결과를 feedback으로 보내고 최종 결과를 result로 전송
## Node2: Subscriber, Service Client, Action Client
- Topic Subscriber : Node1에서 발행하는 학번을 구독
- Service Client : 두 정수를 Node1의 service server에 요청, 곱셈 결과를 수신
- Action Client : Node1의 action server에 학번을 전송, feedback과 result를 수신

## Parameter
- 노드마다 적어도 하나의 매개변수를 설정

## Customized Interface
- Customized msg와 srv, action interface를 사용

## Launch File
- 두 노드를 실행하기 위한 런치 파일을 사용
