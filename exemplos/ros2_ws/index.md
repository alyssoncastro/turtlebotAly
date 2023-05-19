<h1>Simulação de robôs móveis com Gazebo</h2>

autor: Alysson Cordeiro.

<h2>Objetivos</h2>
Crie um pacote em ROS capaz de interagir com uma simulação feita no Gazebo de modo que a plataforma simulada do turtlebot3 seja capaz de mover-se de maneira controlada.

---
<h2>Como utilizá-lo?</h2>
Para utilizar a simulação, tu deverás ter em sua máquina certas aplicações, como:

1. Python: Certifique-se de ter o Python instalado no seu sistema. O código foi escrito em Python, portanto é necessário ter o interpretador Python disponível

2. ROS 2: Certifique-se de ter o ROS 2 instalado no seu sistema. O código utiliza a estrutura e as bibliotecas do ROS 2 para comunicação e execução de nodes.

3. Ambiente de desenvolvimento: É recomendado ter um ambiente de desenvolvimento configurado para trabalhar com ROS 2 e Python, como o ROS 2 Humble instalado em uma distribuição Linux (por exemplo, Ubuntu).

4. Gazebo: Certifique-se de ter o Gazebo instalado no seu sistema. O Gazebo é um simulador de robôs amplamente utilizado no ecossistema ROS. Ele permite simular o comportamento de robôs e ambientes virtuais.

5. Bibliotecas: As seguintes bibliotecas são usadas no código e devem estar instaladas:

*  Rclpy: É a biblioteca para desenvolvimento de nodes em ROS 2 com Python. Pode ser instalada usando o comando pip install rclpy.

* Geometry_msgs: É uma biblioteca que define mensagens de geometria, como Twist e Point, usadas no código. Ela faz parte do ROS 2, portanto não é necessário instalar separadamente.

* Nav_msgs: É uma biblioteca que define mensagens de navegação, como Odometry, usadas no código. Ela também faz parte do ROS 2.

* tf_transformations: É uma biblioteca para converter representações de transformações em ROS. Pode ser instalada usando o comando pip install tf-transformations.
time: É uma biblioteca padrão do Python e não requer instalação adicional.

* Math: Também é uma biblioteca padrão do Python e não requer instalação.


Uma vez que você tenha atendido a esses requisitos, você pode executar o código utilizando o interpretador Python e o ROS 2. Certifique-se de seguir as instruções apropriadas para compilar e executar o código em um ambiente ROS 2.

---
<h2>Passo a passo! </h2>

1. Abra o terminal (ambiente Ubuntu 22.04).

2. Tu recisas chegar no diretório correto para poder executar o ambiente de desenvolvimento: cd ros2_ws/src.

No meu está assim:
* ~/turtleBot/models/m6-ec-encontro1/exemplos/ros2_ws/src/alyPacote/alyPacote$

3. Abra o terminal parelelo ao que já está aberto (ambiente Ubuntu 22.04) e coloque por: gazebo --verbose.

4. Ao abrir o gazebo, vá em "Insert > Add Path > TurtleBot > Models" e aperte em "Choose". Selecione "TurtleBot3 (Burguer)". *obs: clique para ver as imagens*

[image do gazebo aberto](./exemplos/ros2_ws/img/gazebo.png)

[imagem do robo selecionado](./exemplos/ros2_ws/img/lide.png)

5. Agora com o Gazebo aberto. Vamos rodar esse código:

[imagem do código em Python](./exemplos/ros2_ws/img/carbon.png)

6. Para tu rodá-lo, no terminal onde está: *"~/turtleBot/models/m6-ec-encontro1/exemplos/ros2_ws/src/alyPacote/alyPacote$"*, digite: *python3  __init __.py*

[imagem do init](./exemplos/ros2_ws/img/init.png)

Agora digite as coordenadas. 
PRONTO! ESTÁ RODANDO.

<h2>Vídeo</h2>

https://drive.google.com/file/d/1SQhdTuvc0Ox6OiA_pJSyJdju6M4Zg-le/view?usp=sharing

---
---

<h2>Agradecimentos</h2>

Agradecimentos especiais a:

Kil Texeira, que sempre tá me ajudando.

