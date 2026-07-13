| Tested Targets | ESP32-S3 |

# Tarefa 3 - MCC2

## Medições de WCET (osciloscópio)


<img src="img/TEK0002.png" width="60%" />

WCET medido do ir_line task - 5.3uS


<img src="img/TEK0001.png" width="60%" />

WCET medido do speed_ctrl task - 280uS


<img src="img/TEK0000.png" width="60%" />

WCET medido do wheel task- 316uS


<img src="img/TEK0003.png" width="60%" />

WCET medido do treeeyes task - 2.84mS


## Análise de escalonabilidade (RMA)

U = Σ (Cᵢ / Tᵢ) ≤ n·(2^(1/n) − 1)

| Task       | C (WCET)* | T (período) | C/T     |
|------------|-----------|-------------|---------|
| ir_line    | 6.36 µs   | 20 ms       | 0.03 %  |
| speed_ctrl | 336 µs    | 30 ms       | 1.12 %  |
| wheel      | 379 µs    | 50 ms       | 0.76 %  |
| treeeyes   | 3.41 ms   | 60 ms       | 5.68 %  |
| **Total**  |           |             | **U ≈ 7.6 %** |

\* Aos valores de WCET medidos no osciloscópio foi adicionada uma margem de segurança de 20%, pois o máximo observado não é o WCET garantido (cache misses, acessos à flash e interrupções podem aumentá-lo em casos raros).

Como U ≈ 7.6 % é muito menor que o limite de Liu & Layland para n=4 (75.7 %), o sistema é escalonável com grande folga.

## Prioridades de execução (do maior para o menor):
1. ir_line 
2. speed_ctrl
3. wheel
4. treeeyes

Períodos de ativação das tarefas foram escolhidos de acordo com cinematica do robô e funcionamento do PID - por exemplo

## Diagrama UML de comunicação entre as tasks 

<img src="img/diagrama_tasks.png" width="100%" />

- `ir_line_ctrl` e `treeeyes` são os **produtores**: escrevem o estado dos sensores no event group `xEvents`.
- `speed_ctrl` é o **consumidor** do `xEvents`: lê os bits, calcula o PID e envia o comando de duty ao `wheel_ctrl` via task notification (valor de 32 bits: 16 bits por roda, com offset +1024 para transmitir o sinal).
- `wheel_ctrl` faz poll da notificação com timeout 0 (mantém-se estritamente periódico); sem notificação nova, reaplica o último comando.
- A ISR do TWDT apenas **notifica** o `wdt_handler` (trabalho pesado não é permitido em ISR); o handler suspende `speed_ctrl`/`wheel_ctrl` e gira o robô em torno do próprio eixo.
