# 예전 최적화 모델 (사용하지 않음)

`regr_model.joblib`(MultiOutput RandomForest, 회전수·이송 → 12차원 STFT 특징)과 `lof_model.joblib`(LocalOutlierFactor)은
저장소에 없는 `stft_dataset_3axis.csv`(112행)로 학습된 예전 최적화 모델입니다. 2026-10-31부터 프로그램에서 쓰지 않습니다.

- 특징값의 단위가 현재 센서(MPU6050, g 단위) 파이프라인과 수백~수억 배 달라, 실시간 특징을 넣으면 LOF 점수가 모두 약 294(기준 1.5)로
  '안전 후보 없음'이 되어 자동 재가공이 시작되지 않았습니다.
- 안정·채터 라벨 없이 학습되어 '안전'이 '본 적 있는 진동'이라는 뜻이었고, 결과가 구조적으로 "이송 +3%" 아니면 "후보 없음"이었습니다.

자세한 분석은 [edit_analysis.md](../../edit_analysis.md) 2.7, 대체 방법은 `src/optimizer.py`와 `scripts/train_models.py`를 참고하세요.
비교·발표 자료용으로만 보관하며, scikit-learn 1.9.1로 다시 저장된 상태입니다 (원본은 커밋 `861c3b2`).
