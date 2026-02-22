# git-push

변경사항을 커밋하고 GitHub에 push합니다.

## 언제 사용하나

- 코드/문서 수정 후 GitHub에 올릴 때
- 사용자가 "커밋해", "푸시해", "깃에 올려" 등 요청 시

## Steps

1. **상태 확인**
   ```bash
   cd ~/space_ros_ws/src/orbit_sim
   git status
   git diff --stat
   git log --oneline -3
   ```

2. **스테이징**
   - 변경된 파일을 개별적으로 `git add` (`.env`, 인증 파일 등 민감 파일 제외)
   - `git add -A` 사용 금지

3. **커밋 메시지 작성**
   - 형식: `<type>: <한국어 설명>`
   - type: `feat`, `fix`, `docs`, `refactor`, `test`, `chore`
   - HEREDOC 사용하여 커밋:
     ```bash
     git commit -m "$(cat <<'EOF'
     <type>: <설명>

     Co-Authored-By: Claude Opus 4.6 <noreply@anthropic.com>
     EOF
     )"
     ```

4. **Push**
   ```bash
   git push origin main
   ```
   - reject 시: `git pull origin main --rebase` 후 충돌 해결 → 재 push
   - force push 금지 (사용자 명시적 요청 시에만)

5. **결과 보고**
   - 커밋 해시, 변경 파일 수, push 성공 여부

## Git 설정 (초기 1회)

```bash
git config user.name "ndh8205"
git config user.email "ndh8205@users.noreply.github.com"
```

- Remote: `git@github.com:ndh8205/mysrc_sprs_backup.git` (SSH)
