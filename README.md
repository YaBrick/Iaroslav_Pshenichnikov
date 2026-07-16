## *Prof. Matheus Leitzke Pinto*

---

### Objetivo geral

A tarefa tem como objetivo a avaliação das capacidades em desenvolvimento de uma aplicação embarcada com RTOS, considerando a modularização de um problema através de tarefas concorrendo ao uso do processador. Para tanto, deverá ser utilizado o robô Juca, programando em ESP32-S3 diretamente com a API ESP-IDF em ambientes de programação adequados (VScode com plug-in da Espressif ou Espressif IDE) para que o mesmo realize o seguimento de linha em pista especifica disponibilizada pelo professor e com capacidade de evitar colisão de obstáculos.

---

### Objetivos específicos

- Percorrer uma volta completa da pista, sem se desviar da linha.
- Parar ao encontrar um obstáculo 10 cm a sua frente, por meio dos sensores ultrassônicos.
- Continuar o percurso quando obstáculo for retirado.
- Implementar ao menos três tarefas de tempo real, com funcionalidades bem definidas.

---

### Critérios de avaliação e apresentação da tarefa

O aluno deverá apresentar em dia especificado, o robô operando e explicar o código desenvolvido no computador em frente ao professor. Não é para ser apresentado slides, relatório, nem nada semelhante. O aluno deverá criar um fork de https://github.com/MatheusPinto/Tarefas-Microcontroladores-2---2026-1.git, colocando como nome do fork, o seu primeiro e último nome (ex.: matheus pinto). Nesse fork, deverá criar uma pasta com seu primeiro nome e último sobrenome, separados por sublinhado (ex.: matheus_pinto), e deve inserir a pasta do projeto (VScode com plug-in da Espressif ou Espressif IDE) com o nome “tarefa_1”. A arvore de diretórios do fork deverá ter este aspecto:

— Tarefas-Microcontroladores-2---2026-1
     |— matheus_pinto
           |— tarefa_1
                 |— main
                 |— build
                 |— managed_components
                 |— CMakeLists.txt
                 |— …

> [!IMPORTANT]
> **O projeto nesse fork não será avaliado após a data de apresentação. Dessa forma, a atualização do fork deverá ser feita até no máximo, no dia da apresentação!**

Os critérios avaliados e seus pesos serão:

- Estabilidade em seguir a linha (**3 pontos**).
- Qualidade do software (sem código morto, comentários, indentação, etc. — **2 pontos**)
- Clareza e domínio da explicação do código e do que foi desenvolvido (**1 ponto**).
- Uso adequado do RTOS (tamanho de pilhas, definição de prioridades, sem utilizar variáveis globais, comunicação das tarefas baseada e suspensão, etc — **1 ponto**).
- Modularidade das funcionalidades entre as tarefas (ex.: uma tarefa que apenas pisca um LED e outra que lê três sensores, aplica um filtro e controla um motor, é um exemplo de má distribuição de funcionalidades — **3 pontos**).

É importante reforçar algo que foi dito repetidas vezes em aula:


> [!IMPORTANT]
>**O robô operar parcialmente NÃO é garantia de aprovação.**


Um robô pode funcionar com um código simples, sequencial ou improvisado, inclusive com soluções típicas de **Microcontroladores 1**. Porém, esse tipo de desenvolvimento NÃO tem valor avaliativo para a disciplina.

Dessa forma, **se o robô funcionou, mas o código não atende aos critérios técnicos exigidos, isso não configura mérito suficiente para aprovação**, ainda que o resultado físico pareça aceitável.

Outro ponto importante a ser esclarecido:

> [!IMPORTANT]
>**Esforço, dedicação ou tempo investido NÃO são critérios DIRETOS de avaliação.**


Esses aspectos são importantes do ponto de vista pessoal e formativo, mas a avaliação acadêmica se baseia nos requisitos técnicos atendidos, na correção conceitual e na qualidade da solução apresentada. Embora o esforço possa ser considerado no contexto da avaliação, esse esforço só tem valor acadêmico quando TODOS os aspectos abaixo são cumpridos:

- Se implementou corretamente/efetivamente a maior parte do projeto;
- Se apresentou tentativas técnicas válidas, ainda que incompletas;
- Demonstrou entendimento conceitual, mesmo sem sucesso total;
- O número de requisitos não atendidos é pequeno.

Esforço que não se traduz em aplicação correta dos conceitos, metodologias e critérios técnicos definidos para o projeto NÃO pode ser convertido em nota, sob pena de descaracterizar os objetivos da disciplina e comprometer a isonomia do processo avaliativo.

Quanto à apresentação do código e dos resultados, embora a clareza e domínio da explicação do desenvolvimento da tarefa sejam parte do critério avaliativo, **se o aluno demonstrar desconhecimento, ou conhecimento muito raso, do que é apresentado, a avaliação do trabalho será NULA**. Dessa forma, é importante não copiar de forma irresponsável trabalho desenvolvido por terceiros e usar ferramentas de geração de código automático.

---

### Prazos

- Desenvolvimento: 1 de Abril à 15 de Abril
- Apresentação da tarefa: 16 de Abril

---

### Método de avaliação utilizados pelo professor

Para realizar a avaliação de maneira justa, criteriosa e técnica, serão utilizadas **diversas ferramentas e estratégias**, entre elas:

- ferramentas automáticas de **detecção de plágio**;
- análise assistida por **ferramentas de IA**;
- leitura manual e criteriosa do **código-fonte**;
- análise de **arquitetura, originalidade e organização do software**;
- verificação do **atendimento aos requisitos técnicos exigidos**;
- comparação entre **o código entregue** e **o comportamento real do robô**;

De forma pedagógica, a avaliação individuais serão de acesso a todos da turma, SEM identificar os nomes dos alunos. Dessa forma, todos irão entender como o processo de avaliação foi conduzido para a análise e refinamento das próximas tarefas.