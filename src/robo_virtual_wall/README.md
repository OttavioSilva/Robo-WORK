# Visão de parede virtual e integração Nav2 — ROS 2 Jazzy

Reaproveita o algoritmo de `~/Downloads/detecta_parede_virtual.py`: HSV com
wrap de vermelho, abertura 3x3, fechamento 25x25, área, alongamento, listras e
fração branca. O arquivo original e suas calibrações não são modificados.
O detector não publica velocidade nem goals. A integração opcional publica uma
máscara de keepout para Nav2; ver [NAV2.md](NAV2.md).

## Executar no terminal do computador

Feche o detector original antes de abrir a câmera neste nó. Dependências estão
no package.xml; não foram instaladas automaticamente. Compile no workspace ROS
que contém este pacote (o workspace operacional pode diferir deste checkout).

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robo_virtual_wall
source install/setup.bash
ros2 launch robo_virtual_wall vision.launch.py show_windows:=true
```

Padrão: `/dev/video2`, V4L2, MJPG, 640x480, 30 FPS solicitados, processamento
15 Hz. A resolução/FPS negociados são registrados. Não altera balanço de branco
ou exposição: diferentemente do script original, evita mudar controles físicos
silenciosamente. Verifique HSV sob os controles atuais. Para câmera publicada
por driver ROS, use `image_topic:=/camera/image_raw`; nesse modo não abre V4L2.
Use imagem raw da mesma câmera e resolução da calibração. Feche com q ou Ctrl+C.

Lê HSV de `~/Downloads/hsv_fita.json` ao iniciar; valores inválidos impedem
inicialização. Calibração continua manual no script original; faça backup antes
de salvar. Não existe gravação automática de HSV neste pacote.

Tópicos:
- `/virtual_wall/image`: imagem anotada, sensor_msgs/Image bgr8.
- `/virtual_wall/mask`: máscara unida, Image mono8; pode conter rejeitados.
- `/virtual_wall/detections`: String JSON com timestamp, pixels e, quando válidos,
  extremos métricos em base_link.
- `/virtual_wall/segments`: MarkerArray para RViz, Fixed Frame base_link ou um
  frame conectado por TF. Observações expiram em 1 s; não são mapa persistente.

`roi_top` padrão 0.10 exclui o topo. `roi_bottom` padrão 0.85 exclui a
faixa inferior onde aparece o próprio robô; por exemplo `roi_bottom:=0.65` deve ser avaliado visualmente,
não assumido como correto. O corte é reaplicado após fechamento para evitar
reintroduzir pixels excluídos. Não há confirmação temporal de identidade da fita.

## Projeção métrica, desabilitada por padrão

`homography_file` vazio publica somente pixels. O formato legado contendo apenas
H não é aceito: falta confirmar origem, resolução, domínio e validade física.
Um arquivo validado deve conter:

```json
{
  "frame_id": "base_link", "units": "m", "image_space": "raw",
  "validated": true, "image_size": [640, 480],
  "H": [[1,0,0],[0,1,0],[0,0,1]],
  "valid_pixels": [[100,200],[540,200],[540,470],[100,470]]
}
```

**Esquema ilustrativo: H identidade acima NÃO é uma calibração utilizável.**
Não renomeie a calibração antiga nem marque validated sem medir. H deve mapear
pixels raw para chão em metros, x frente e y esquerda, origem base_link. Meça
pontos distribuídos pelo chão; confira com pontos independentes não usados no
ajuste. Quatro pontos podem dar erro de ajuste zero sem precisão fora deles.

Homografia assume plano e montagem fixa (altura/inclinação). Valide erro de lente
na região utilizada; se significativo, calibre intrínsecos e distorção e implemente
um pipeline consistente de retificação + nova homografia antes de habilitar
metros. Este adaptador não aplica correção de distorção e não aceita image_space
rectified. Mudança de câmera, foco, montagem ou resolução requer revalidação.
Recorta segmentos observados ao polígono convexo calibrado, sem cruzar o
horizonte, à frente e dentro de max_range_m (3 m). Não extrapola trechos ocultos.

## Integração Nav2

Implementada em `tape_keepout`, com KeepoutFilter e inflação nos costmaps global e
local, habilitados por `virtual_walls:=true`. Ver [NAV2.md](NAV2.md) para calibração,
ativação e limites. Nenhuma calibração preliminar foi promovida automaticamente.

## Ajuste visual no próprio nó

```bash
ros2 launch robo_virtual_wall vision.launch.py show_windows:=true calibrate:=true
```

A janela `ajuste HSV` controla a segmentação em tempo real. `mascara vermelha`
mostra a seleção anterior ao fechamento; `mascara unida` mostra a união e a imagem
anotada mostra rejeições/quantidade aceita. Ajuste até selecionar as listras ao
longo da fita, não apenas um trecho. Os limiares padrão continuam os originais.
Somente a tecla `s` grava HSV, após criar backup exclusivo com data/hora;
`q` encerra sem salvar ajustes pendentes. Os filtros geométricos permanecem iguais.
Não use persistência visual para esconder perdas de detecção: estabilize primeiro
cor/iluminação e valide os critérios de padrão com a fita real.

## Faixas largas e chevrons vermelhos

O alongamento mínimo passou de 4 para 2 para incluir placas/faixas largas.
Quando há menos de cinco componentes vermelhos separados, um teste adicional
amostra sete linhas no eixo maior do retângulo rotacionado e exige pelo menos
cinco trechos vermelhos e quatro brancos, de dois pixels ou mais, em duas linhas.
Isso permite chevrons com braços unidos. A fração branca e a área mínima continuam
obrigatórias. Amarelo/preto não foi incluído: esta variante continua vermelho/branco.
A união morfológica pode agrupar duas faixas próximas; o segmento publicado nesse
caso representa o grupo observado, não necessariamente duas paredes distintas.
Testes sintéticos cobrem chevrons conectados, faixa larga, vermelho sólido e
rejeição de amarelo. Ainda requer validação física em perspectiva e sob iluminação
variável; não é classificação semântica garantida.

## Junções próximas (T/L)

Contornos unidos são examinados com propostas de segmentos Hough nas bordas da
máscara. Cada proposta precisa passar pelos filtros vermelho/branco em uma faixa
local; direções distintas são mantidas em vez de uma única reta média. O ajuste
original permanece quando não há ao menos dois segmentos validados. Não completa
a fita em regiões ocultas. Testes sintéticos cobrem T e encontro diagonal; junções
reais, faixas paralelas muito próximas e mudanças de escala ainda exigem validação.
O custo de processamento aumenta em imagens com muitas bordas candidatas.

## Estabilidade visual

As propostas Hough agora são estendidas pela projeção do contorno observado antes
da validação, evitando restringir uma fita longa ao primeiro fragmento encontrado.
A linha verde exibida usa suavização exponencial (alpha 0.45), associando direção
e extremos próximos, inclusive quando a ordem dos extremos se inverte. Não mantém
segmentos ausentes e reinicia após pausa de processamento superior a 0.5 s.
JSON e marcadores métricos continuam usando medidas atuais, sem suavização visual.
Isso reduz tremulação de coordenadas; não garante eliminar perdas reais de máscara.

Propostas sobrepostas são consolidadas antes da publicação e da suavização: o
segmento menor é descartado se ambos os extremos ficam a até 25 pixels do eixo
maior, a diferença angular é até 20 graus e há sobreposição de ao menos 60% do
comprimento menor. Isso evita contar bordas como faixas adicionais. Ramos
perpendiculares são preservados; faixas paralelas separadas por menos de 25 pixels
podem ser confundidas e exigem validação/ajuste de escala. Não há histerese artificial
do contador. Testes de junção agora exigem exatamente dois segmentos sintéticos.

## Prévia a partir das quatro cruzes medidas

`preview_homography_file` aceita homografia de frame `virtual_wall_front` apenas
para anotação da imagem. Não habilita medidas JSON nem marcadores métricos.
A amostra em calibration/front_preview_20261106_123827.json foi calculada com
centros aproximados das cruzes numa captura de tela 640x480 sem redimensionamento,
referência no centro da borda frontal branca, X frente e Y esquerda. Está marcada
validated=false. A região interpolada abrange somente o quadrilátero das quatro
cruzes (20–45 cm, laterais ±15 cm). A prévia exibe o menor X do trecho de fita
observado dentro desse domínio, não distância radial nem garantia de folga de
todo o footprint. Testar fita transversal a 30 cm com trena, depois outros pontos
independentes; não usar ajuste nos mesmos quatro pontos como prova de precisão.
Mudança de câmera/montagem invalida a homografia. Não habilitar ROS métrico sem
calibração validada, avaliação de distorção e transformação da referência frontal
para base_link.

### Distância comparável à trena na linha central

A prévia agora substitui o menor X do trecho pela interseção exata do segmento
observado com Y=0 na referência frontal. Mostra `Eixo da fita na linha central`
e um ponto amarelo no cruzamento. Essa é a distância longitudinal da borda frontal
até o eixo estimado da fita, não até sua borda próxima nem folga do footprint.
Entre vários cruzamentos exibe o mais próximo à frente. Só mede cruzamentos dentro
do segmento observado e do quadrilátero calibrado; não prolonga segmentos para
inventar interseções. Faixas paralelas/coincidentes com a linha central não dão
cruzamento único. Sem cruzamento válido exibe explicitamente a ausência de medida.
Não houve compensação de 1.5 cm nem alteração da homografia com base no teste da
trena. Compare o novo valor com os 36 cm medidos ao eixo; ainda é uma prévia.

### Prévia: orientação e continuidade da leitura

A prévia classifica os segmentos no plano do chão, recortando o suporte observado
pelo polígono calibrado. Fitas a até 20 graus do eixo longitudinal mostram o
deslocamento lateral médio do trecho calibrado (positivo à esquerda), em vez de
uma distância de cruzamento mal condicionada. Esse valor não é folga do robô.
Fitas fora do polígono continuam detectáveis e recebem mensagem explícita; a
homografia não é extrapolada além das referências medidas.

A distância transversal exibida usa mediana dos últimos três valores consecutivos.
Perda do cruzamento limpa o histórico imediatamente; saltos maiores que 3 cm ou
intervalos de processamento acima de 0,5 s também reiniciam o filtro. O ponto
amarelo e as publicações mantêm os dados atuais, sem essa filtragem. Um candidato
validado de corredor também é preservado quando o contorno global falha na
validação, permitindo recuperar uma fita dentro de grupos misturados.

Para a montagem reposicionada em 06/11, usar
`calibration/front_preview_20261106_repositioned.json`. Continua preliminar:
conferência relatada em cerca de 35,5 cm, com leitura de 34,8–35,2 cm. Para
ampliar o domínio ou validar precisão em toda a área, medir novas referências
independentes com a câmera fixa.

A conexão das listras inclui fechamentos direcionais estreitos (0/45/90/135 graus),
com alcance aproximado de 40 pixels. Fragmentos já validados podem ser unidos
entre contornos com diferença angular até 5 graus, distância lateral até 8 pixels
e intervalo até 40 pixels. Não extrapola extremos externos observados. Intervalos
curtos entre fitas distintas alinhadas também podem ser unidos; testar ao vivo.
Com a janela selecionada, `r` salva o quadro original sem desenhos e o HSV/ROI em
`~/Robo-WORK/camera_captures/`, para reproduzir falhas sem interferência da sobreposição.

### Distância na prévia 1280x960

`preview_allow_resize:=true` permite reescalar somente a prévia quando a proporção
é idêntica (640x480 → 1280x960). H é composta com a escala inversa dos pixels e o
polígono acompanha a nova resolução. A tela mostra `PREVIA REESCALADA`: isso assume
mesmo campo de visão, câmera/foco fixos e exige conferência com trena. Mesma proporção
não prova mesmo enquadramento do sensor. Mudança para 16:9 é rejeitada. Não promove
calibração nem gera `base_link_m`; a projeção Nav2 mantém resolução estrita. Caso o
modo da câmera recorte ou altere o enquadramento, refazer a homografia com pontos
medidos na resolução nova. Distância/polígono também aparecem no tópico de imagem,
mesmo sem janelas. O histórico de distância reinicia quando a resolução muda.

### Conferir pontos clicando na câmera

Com `show_windows:=true` e `preview_homography_file` carregado, clique com o botão
esquerdo no centro de uma marca no chão dentro da área azul. A cruz magenta mostra
o pixel selecionado. O texto `CLIQUE` informa frente e lateral em centímetros na
referência da borda frontal branca. Não é distância radial. Fora do domínio não
há extrapolação. Botão direito limpa a seleção. A seleção é fixa em pixels, não
segue objetos nem corrige movimento da câmera/robô. Mudança de resolução a limpa.

Cruzes usadas no ajuste recebem aviso explícito e não podem ser salvas como
verificações independentes. Para comparar um ponto novo com trena: selecione-o,
pressione `M`, digite dois números em centímetros separados por espaço e pressione
Enter. Primeiro frente a partir da borda branca, depois lateral a partir da linha
central, positiva à esquerda e negativa à direita. Exemplo: `35 -10` significa
35 cm à frente e 10 cm à direita. Esc cancela; Backspace apaga. O ponto medido deve
estar no chão; não clicar em objetos elevados.

Cada Enter grava a lista de verificações da sessão em
`~/Robo-WORK/camera_captures/checks-*.json`, contendo pixels, medidas reais, projeções,
erros e resolução. O terminal informa o arquivo. Esse arquivo é compatível com
`validate_tape_calibration --checks`, que ainda exige distribuição e erro adequados;
não habilita Nav2 automaticamente. A foto `R` inclui o ponto selecionado nos
metadados JSON. Conferir cliques em cruzes de ajuste não comprova precisão: usar
marcas novas para os testes independentes. Manter robô/câmera fixos durante a sessão.

### Eixo de faixas finas e teclas

Contornos já validados como fita, com alongamento de ao menos 6 e espessura até
35 pixels, usam diretamente um eixo central único. A busca Hough por múltiplas
direções fica para grupos mais largos/complexos; isso evita que bordas das listras
se tornem duas fitas em faixas finas. O eixo é ajustado com Huber aos centros de
seções longitudinais do contorno, reduzindo o peso da amostragem irregular das
bordas. Testes de junções T/diagonais e fitas paralelas permanecem obrigatórios.
Esses limites são em pixels e devem ser reavaliados para outras resoluções.
As teclas R/M/S/Q também aceitam maiúsculas. Uma captura sem sobreposição continua
necessária para reproduzir episódios reais de duplicação que os testes não cobrem.

### Regressão real: fio vermelho do próprio robô

Nas capturas `raw-20261106-202400-368515.png` e `raw-20261106-202411-842372.png`, a
máscara podia conectar a fita longitudinal ao fio vermelho sobre o robô. O grupo
resultante gerava uma segunda direção diagonal falsa. O corte inferior padrão
passa a `roi_bottom=0.85` (408 pixels em 480), também explícito no launch operacional.
A região calibrada termina em y=385 pixels e permanece inteira. O corte é aplicado
antes/depois da morfologia. Os dois quadros originais estão em `test/fixtures` e
precisam produzir exatamente uma fita, com eixo próximo de x=274 pixels.

Esse corte é específico ao enquadramento atual. Se a montagem mudar, reavaliar
ROI/calibração. Não identifica nem elimina objetos vermelhos em toda a imagem;
impede esta contribuição observada do próprio robô. Conferir ao vivo a continuidade
em ângulos diferentes. Outros episódios de duplicação precisam de novas capturas.

### Estabilidade do eixo nas junções reais

Nos dois quadros `two_real_tapes_a/b.png`, a proposta Hough podia alternar entre
bordas diferentes da fita longitudinal. Cada direção aceita agora é recentrada
por seções longitudinais usando as duas bordas da máscara unida. Seções largas
ou cortadas pelo corredor de busca são ignoradas, para que a faixa transversal
não puxe o ajuste lateralmente. O ajuste robusto Huber conserva apenas suporte
observado, inclusive na extensão dos extremos.

Na comparação desses dois quadros, a diferença lateral calculada em X=0,70 m de
base_link caiu de cerca de 1,6 cm para 2,6 mm. A posição longitudinal da faixa
transversal ficou igual nos dois quadros após o refinamento. Isso é uma regressão
sobre duas imagens, não uma medição de estabilidade temporal ao vivo. O teste
exige duas fitas e diferença inferior a 5 mm nesse par; movimento e TF reais
continuam exigindo conferência no robô.

## Captura contínua, reconexão e paredes permanentes (21/11/2026)

- `camera.py` (CAMERA_RECONEXAO_20261121): uma thread lê a XIFT na taxa da câmera
  e guarda só o quadro mais novo (`CAP_PROP_BUFFERSIZE=1`). A hora da imagem é a da
  aquisição: hora da leitura menos a idade do buffer V4L2, usada só se ficar entre
  0 e `max_driver_age_s` (0,5 s), o que pressupõe o driver em CLOCK_MONOTONIC.
  Fora disso, usa a hora da leitura e avisa. `CAMERA_LATENCIA` no log a cada 10 s
  mostra idade no driver e tempo de processamento (mediana/p90).
- Sem quadros por `reconnect_after_s` (2 s), a câmera é fechada e reaberta pelo mesmo
  caminho (`/dev/v4l/by-id/...`), reaplicando `v4l2_controls` (foco) e o formato.
  Só a captura reinicia; o `tape_keepout` e as paredes confirmadas continuam.
- SEM_RETIRADA_20261121: `observation_tape.launch.py` agora usa
  `remove_visible_missing:=false` por padrão. Fita confirmada fica no mapa até
  `ros2 service call /tape_keepout/clear std_srvs/srv/Empty`. Use `true` só no teste
  de retirada.
- DETECTOR_RECORTE_20261121: `parece_fita` trabalha num recorte em volta do contorno.
  Saída idêntica em 21 imagens/922 chamadas; mediana 118→89 ms, p90 229→171 ms.

## Calibração ChArUco com correção de lente (21/11/2026)

`~/calibrar_camera.sh` (ferramenta Tk sem ROS, em `ferramentas/calibracao/`). PDFs em
`ferramentas/calibracao/imprimir/` (`lente_A4.pdf`, `chao_A3_4_folhas.pdf`; imprimir a 100%).

1. **Lente:** tabuleiro A4 na mão, fotos automáticas quando parado; `calibrate_lens` com
   descarte de fotos ruins. Gera `calibration/lente_xift_*.json` (uma vez por câmera).
2. **Chão:** folhas A3 no chão; trena só nos cantos A e B de cada folha (frente da borda
   branca, lateral do centro, + esquerda). PnP conjunto dá a pose da câmera; H vale para todo
   o chão visível. Área válida = pixels com resolução ≤ 3 cm/px, alcance ≤ 3 m do base_link,
   à frente da borda e acima de 85% da altura da imagem. Gera `front_charuco_*.json`
   (`image_space: undistorted`, `camera_matrix`, `dist_coeffs`) e a referência de montagem.
3. **Conferência:** ≥ 6 cruzes novas (perto e longe obrigatórios), tolerância padrão 2 cm;
   promove para `front_base_link_charuco_*.json` e atualiza o `parede.sh` (com backup).

CAMERA_MEXEU: calibrações com `mount_reference` comparam a cada 2 s dois recortes do corpo do
robô (cantos inferiores). Deslocamento > 3 px ou rotação ≥ 1° em 3 verificações seguidas
suspende as medidas em metros (`perception_valid=false`); canto coberto = inconclusivo.
Validado com fotos reais de 13/11 (a câmera foi reposicionada às ~21:25 daquele dia).
