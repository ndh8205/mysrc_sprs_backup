# audit-docs

프로젝트 CLAUDE.md와 README.md를 점검하고 현재 파일시스템과 불일치하는 부분을 찾아 최신화합니다.

## 언제 사용하나

- 세션 시작 시 프로젝트 문서 상태를 빠르게 확인할 때
- 노드, launch 파일, 모델이 추가/삭제된 후 문서 동기화가 필요할 때
- 빌드 후 setup.py entry_points와 실제 노드 파일 불일치 의심 시

## Steps

1. **파일 목록 수집**
   - `orbit_sim/orbit_sim/*.py` — Python 노드 목록
   - `orbit_sim/launch/*.launch.py` — Launch 파일 목록
   - `orbit_sim/models/` — Gazebo 모델 목록
   - `orbit_sim/worlds/` — World 파일 목록

2. **문서 대조 검증**
   - CLAUDE.md의 디렉토리 구조 → 실제 파일시스템
   - README.md의 패키지 구조 → 실제 파일시스템
   - setup.py entry_points → 실제 노드 파일 존재 여부
   - package.xml 의존성 → 실제 import 사용

3. **불일치 보고서 작성**

   ```
   ## 문서 감사 결과 — YYMMDD

   ### 정상 (N개)
   - [파일]: 모든 항목 일치

   ### 불일치 발견 (N개)
   | 파일 | 이슈 | 심각도 |
   |------|------|--------|
   | CLAUDE.md | 노드 수 불일치 (문서: 21, 실제: 23) | MEDIUM |
   | setup.py | entry_point 누락: new_node | HIGH |
   ```

4. **사용자 확인 후 수정 진행**
   - HIGH: 빌드/실행에 영향 → 즉시 수정 제안
   - MEDIUM: 문서 정확성 → 확인 후 수정
   - LOW: 보고만
