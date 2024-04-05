#!/bin/bash

# 첫 번째 인자를 파일 이름으로 사용
filename="$1.bash"

# touch 명령어로 파일 생성
touch "$filename"

# chmod 명령어로 사용자에게 실행 권한 부여
chmod u+x "$filename"

echo "$filename 생성 및 실행 권한 부여 완료"


