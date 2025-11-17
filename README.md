1) task/start (std_msgs/Int32)

체크박스 선택 상태에 따라 다음 값을 전송합니다:

선택 조합	의미	전송 값(Int32)
아무것도 선택 X	작업 없음	0
Roller만 선택	Roller 작업	1
Cloth만 선택	Cloth 작업	2
Roller + Cloth 선택	두 작업 동시	3

▶ START 버튼을 누른 직후 전송됨.

2) task/end (std_msgs/Bool)
상태	전송 값
END 버튼 클릭	True
START 버튼 클릭	False
E-STOP 클릭	False

END 버튼을 누르면:

task/end = True

그리고 start 값은 0으로 초기화(Int32=0)

3) task/emergency_stop (std_msgs/Bool)
상태	전송 값
E-STOP 버튼 클릭	True
START 버튼 클릭	False
END 버튼 클릭	False

E-STOP 클릭 시:

start: 0

end: False
