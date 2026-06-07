@ donginC_protocol.py
 shortest_delta_u14 <- 이거 필요?


 RobotController가 RobotStateC를 직접 구성함. robot_controller/telemetry 제거 완료.

 내가 봤을때 치명적인 부분들은 각 서브 프로세스들이나 단위 클래스들이 자신들 만의 데이터 클래스를 각각 따로 가지고 있는 부분 같거든? 이거는 shm 아래에 통합 시켜야할 것 같은데 어떻게 생각해    


프로젝트의 쿼터니언 표현방식을 wxyz로 통일
