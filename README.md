## Tabela 1 — Single Core

| Métrica | `mpu_task` | `fusion_task` | `uart_task` | `pwm_task` |
|---|---|---|---|---|
| WCET | 423.2 µs | 13.2 µs | 1002 µs | 0.2 µs |
| Jitter | 800 ns | 20.0 ms | 997.8 µs | 18.1 µs |
| Deadline Miss Rate | 0% | 0% | 0% | 0% |
| Stack Usage | 1.37% | 3.71% | 1.27% | 3.52% |

---

## Tabela 2 — SMP (2 Cores)

> Core 0: `mpu_task` + `fusion_task` — Core 1: `uart_task` + `pwm_task`

| Métrica | `mpu_task` | `fusion_task` | `uart_task` | `pwm_task` |
|---|---|---|---|---|
| WCET | 428.9 µs | 25.1 µs | 4000 µs | 0.3 µs |
| Jitter | 700 ns | 5.6 µs | 999.3 µs | 5.0 µs |
| Deadline Miss Rate | 0% | 0% | 0% | 0% |
| Stack Usage | 2.15% | 5.62% | 1.27% | 3.52% |

