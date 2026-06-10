# LG PRI DigitalTwin UI 스타일 가이드

이 스킬은 LG SmartFLO / NuRec 스타일의 엔터프라이즈 웹 UI를 생성할 때 적용하는 디자인 시스템입니다.

## 레이아웃 구조

```
┌──────────────────────────────────────────────┐
│ app-header (48px, #1a1a1a 다크)               │
├──────────────────────────────────────────────┤
│ menu-bar (30px, #f0f0f0)                      │
├──────────────────────────────────────────────┤
│ sub-menu-bar (26px, #f8f8f8)                 │
├─────────────┬──────────────┬─────────────────┤
│ panel-left  │ panel-center │ panel-right      │
│ (260px)     │ (flex:1)     │ (300px)          │
├──────────────────────────────────────────────┤
│ status-bar (22px, #1a1a1a 다크)              │
└──────────────────────────────────────────────┘
```

- `body`: `height: 100vh; overflow: hidden; display: flex; flex-direction: column;`
- `main-content`: `flex:1; display:flex; overflow:hidden; padding:8px; gap:6px; min-height:0;`

## 색상 팔레트

| 용도 | 값 |
|------|----|
| Primary accent | `#c8002f` (LG 레드) |
| Primary dark | `#a8002a` (hover) |
| Header/StatusBar background | `#1a1a1a` |
| Menu bar background | `#f0f0f0` |
| Page background | `#e4e4e4` |
| Panel background | `#ffffff` |
| Panel body background | `#fafafa` |
| Border | `#d4d4d4` |
| Success | `#2d7a2d` |
| Text primary | `#222` |
| Text secondary | `#555` |
| Text muted | `#999` |

## 컴포넌트 스펙

### app-header
```css
background: #1a1a1a;
height: 48px;
display: flex; align-items: center; justify-content: space-between;
padding: 0 14px;
```
- 좌측: LG 로고 `<img src="/lg-logo.png">` + 브랜드명 + 서브타이틀
- 우측: `.header-btn` 버튼 그룹 (활성 버튼은 `background: #c8002f`)

### menu-bar / sub-menu-bar
```css
/* menu-bar */
background: #f0f0f0; border-bottom: 1px solid #ccc; height: 30px;

/* sub-menu-bar */
background: #f8f8f8; border-bottom: 1px solid #e0e0e0; height: 26px;
```
- 활성 탭: `color: #c8002f; border-bottom: 2px solid #c8002f; font-weight: 600;`
- 활성 탭에 `✓` 프리픽스 붙이기

### panel
```css
background: #fff;
border: 1px solid #d4d4d4;
display: flex; flex-direction: column;
```

### panel-header
```css
background: #f5f5f5;
border-bottom: 1px solid #e0e0e0;
border-left: 3px solid #c8002f;   /* ← 좌측 빨간 강조선 */
padding: 5px 10px;
font-size: 12px; font-weight: 600; color: #333;
```

### 버튼
```css
/* Primary (제출/실행) */
background: #c8002f; color: #fff; border: none; border-radius: 2px;
padding: 7px; font-size: 12px; font-weight: 600;

/* Header 버튼 */
background: #2e2e2e; border: 1px solid #3e3e3e; color: #bbb;
padding: 4px 14px; font-size: 12px; border-radius: 2px;

/* 보조 (로그/리셋 등) */
background: none; border: 1px solid #ddd; color: #777; border-radius: 2px;
```

### 입력 탭 (파일/URL 전환)
```css
/* 탭 컨테이너 */
display: flex; border: 1px solid #ddd; border-radius: 2px; overflow: hidden;

/* 비활성 탭 */
background: #f5f5f5; color: #555; font-size: 11px;

/* 활성 탭 */
background: #c8002f; color: #fff; font-weight: 600;
```

### 스텝 리스트 (파이프라인 진행 표시)
```css
/* 기본 */
border-left: 3px solid #eee; background: #fafafa;

/* 활성 (현재 처리 중) */
border-left-color: #c8002f; background: #fff8f8;

/* 완료 */
border-left-color: #2d7a2d; background: #f8fff8;
```
- `.step-num`: 22px 원형, 상태에 따라 border/color 변경
- 완료 시 `✓`, 처리 중 시 CSS 스피너

### 상태 뱃지
```css
.badge-queued  { background: #f0f0f0; color: #777; }
.badge-running { background: rgba(200,0,47,0.09); color: #c8002f; }
.badge-done    { background: rgba(45,122,45,0.09); color: #2d7a2d; }
.badge-failed  { background: rgba(200,0,47,0.09); color: #c8002f; }
```

### status-bar (하단)
```css
background: #1a1a1a; height: 22px;
display: flex; align-items: center; padding: 0 12px;
border-top: 1px solid #2e2e2e; font-size: 10px; color: #666;
```
- 좌측: 서버 연결 상태 dot (green `#4caf50` / red `#c8002f`) + 상태 텍스트
- 우측: 버전 정보 + 시계

### Empty State (패널에 데이터 없을 때)
```css
display: flex; flex-direction: column; align-items: center; justify-content: center;
height: 100%; color: #bbb; gap: 8px; text-align: center; padding: 20px;
```
- 아이콘 (font-size: 36px; opacity: 0.45)
- 제목 (font-size: 12px; color: #999; font-weight: 500)
- 설명 (font-size: 11px; color: #bbb; line-height: 1.6)

## 타이포그래피

- 기본 폰트: `"Segoe UI", -apple-system, BlinkMacSystemFont, sans-serif`
- 기본 크기: `13px`
- 패널 내부 텍스트: `11~12px`
- 로그/코드: `"Consolas", monospace; font-size: 10px`

## 적용 규칙

1. `border-radius`는 `2px` 또는 `3px`으로 최소화 (둥글지 않은 엔터프라이즈 느낌)
2. 그림자 없음 (`box-shadow` 미사용)
3. 애니메이션은 `0.12~0.25s` 짧게
4. 패널 헤더에 반드시 `border-left: 3px solid #c8002f` 적용
5. LG 로고는 `/lg-logo.png` (static 폴더에 위치)
6. 활성 메뉴/탭 항목에 `✓` 프리픽스 표시

## 사용 예시

이 스킬을 호출하면:
- 위 레이아웃과 색상 시스템을 기반으로 UI를 생성하거나 수정합니다.
- 기존 컴포넌트와 일관성을 유지합니다.
- SmartFLO / NuRec 스타일의 엔터프라이즈 웹 UI 표준을 따릅니다.
