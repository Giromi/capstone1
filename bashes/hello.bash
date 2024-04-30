#!/bin/bash

# 현재 디렉토리의 .bash 파일에 대해 반복
for file in *.bash; do
  # 파일이 실제로 존재하는지 확인
  if [ -f "$file" ]; then
    echo "실행 권한 부여: $file"
    chmod u+x "$file"
  fi
done
echo ">>> 모두 완료 <<<"
