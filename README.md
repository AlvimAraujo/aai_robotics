# aai_robotics
Repository to participate on SBAI ROSI challenge.

## Introdução ##
Repositório GitHub que contém um pacote ROS para participar do desafio ROSI no Simpósio Brasileiro de Automação Inteligente (SBAI). O desafio consiste em criar um pacote que contém programas que irão controlar o robô ROSI para cumprir algumas tarefas. 

## Ficha Técnica ## 
Este pacote foi feito com ROS Kinetic Kame e Ubuntu 16.04 LTS, enquanto que o desafio foi preparado em ROS Melodic Morenia e Ubuntu 18.04.

A versão do V-REP utilizada é a mesma que a do desafio (3.6.2, PRO EDU), bem como a comunicação vrep-ros-interface. 
Até então, não houveram conflitos de versões do pacote.

## Launchs ##
Os launchs que contêm as realizações dos desafios são os nomeados como "vX_rosi_challenge.launch". O que deve ser executado é o launch com o maior número no lugar do X, isto é, a última versão do launch que controla o robô para realizar as tarefas do desafio.

Abaixo segue a descrição do que fazem e quais as mudanças em cada launch disponível no pacote.

rosi_teleop.launch --> Movimenta o robô pelo teclado.

rosi_control1.launch --> Recebe do usuário uma coordenada gps (x, y) e manda o robô para ela.

rosi_control2.launch --> Mesmo que o control1, mas utiliza a técnica de planejamento de campos potenciais artificiais. OBS: O campo repulsor não está devidiamente implementado pois precisa da leitura do kinect para a identificação dos obstáculos, a saber, precisa da coordenada do obstáculo mais próximo.

v1_rosi_challenge.launch --> Usa alguns nós para fazer com que o robô complete a tarefa da coleta andando no mapa.

v2_rosi_challenge.launch --> Similar ao v1, no entando, o robô é capaz de ir e voltar em uma parte do mapa por tempo infinito.

v3_rosi_challenge.launch --> Similar ao v2, no entando, pelo sensor hokuyo, o braço ur5 sempre vira para o TC, de modo que a coleta é sempre realizada, independente da orientação do robô. OBS: Há alguns casos excepcionais para serem tratados.
![Relação entre nós e tópcios](images/v3_rqt_graph.png?raw=true "v3_rqt_graph")

## Nodes ##


## Topics ##
Todos os tópicos criados pelos desenvolvedores começam com o prefixo aai, e têm a finalidade de facilitar as stream de dados e filtrar aqueles que são úteis.

aai_rosi_pose -->

aai_rosi_cmd_vel -->

## Equipe ##
Álvaro Rodrigues Araújo;

Arthur Henrique Dias Nunes;

Israel Filipe Silva Amaral.

AAI Robotics - Universidade Federal de Minas Gerais

Apostila de Simulações Robóticas(em desenvolvimento): https://pt.overleaf.com/read/mmvbybzdntcd

![Equipe](images/equipe.jpeg?raw=true "Equipe")
![Logo](images/logo.jpeg?raw=true "Logo")

