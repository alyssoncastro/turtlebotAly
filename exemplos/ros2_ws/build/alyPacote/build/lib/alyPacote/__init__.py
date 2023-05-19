import rclpy  # Importa a biblioteca rclpy para trabalhar com o ROS 2
from rclpy.node import Node  # Importa a classe base Node para criar nós do ROS 2
from geometry_msgs.msg import Twist, Point  # Importa as mensagens Twist e Point
from nav_msgs.msg import Odometry  # Importa a mensagem Odometry
from tf_transformations import euler_from_quaternion  # Importa a função euler_from_quaternion
from time import sleep  # Importa a função sleep para pausar a execução


# Criando classe TurtleController
class TurtleController(Node):
    def __init__(self):
        super().__init__('subscriber_node')  # Chama o construtor da classe base Node
        self.x, self.y, self.theta = 0.0, 0.0, 0.0  # Inicializa as variáveis para armazenar a posição e orientação
        self.point_list = []  # Inicializa uma lista vazia para armazenar as coordenadas da trajetória
        self.condicao = True  # Inicializa a variável de controle
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)  # Cria um publisher para enviar comandos de velocidade
        self.subscription = self.create_subscription(Odometry, '/odom', self.listener_callback, 4)  # Cria uma subscription para receber informações de odometria
        self.timer = self.create_timer(0.02, self.publisher_callback)  # Cria um timer para chamar periodicamente o método publisher_callback


    def listener_callback(self, msg):
        self.x = msg.pose.pose.position.x  # Armazena a posição x da tartaruga
        self.y = msg.pose.pose.position.y  # Armazena a posição y da tartaruga
        _, _, self.theta = euler_from_quaternion([  # Converte a orientação da tartaruga para ângulos de Euler
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w #
        ])


    def publisher_callback(self):
        if self.condicao:
            resposta = input("vamo' iniciar a trajetória? (y/n): ")  # Solicita a entrada do usuário para iniciar a trajetória
            if resposta == 'y':
                valor_coord = float(input("Digite a coordenada aí: "))  # Solicita a entrada do usuário para adicionar uma coordenada à trajetória
                self.point_list.append(valor_coord)  # Adiciona a coordenada à lista de coordenadas
                self.condicao = False  # Define a condição como False para evitar a solicitação novamente
            else:
                self.condicao = True  # Define a condição como True para continuar solicitando

        msg = Twist()  # Cria um objeto da mensagem Twist para enviar comandos de velocidade
        posicao = Point()  # Cria um objeto da mensagem Point para definir a posição
        posicao.x = self.point_list[0] if self.point_list else 0.0  # Define a coordenada x de destino como a primeira coordenada da lista, ou 0.0 se a lista estiver vazia
        caminho_x = posicao.x - self.x  # Calcula a distância restante até o ponto de destino

        if self.x <= posicao.x and caminho_x >= 0.2:  # Se a posição atual é menor ou igual à posição de destino e a distância restante é maior ou igual a 0.2
            msg.linear.x = 0.5  # Define a velocidade linear para frente
        elif self.x >= posicao.x and caminho_x <= 0.2:  # Se a posição atual é maior ou igual à posição de destino e a distância restante é menor ou igual a 0.2
            msg.linear.x = -0.5  # Define a velocidade linear para trás
        else:
            msg.linear.x = 0.0  # Define a velocidade linear como zero para parar
            msg.angular.z = 0.0  # Define a velocidade angular como zero
            if self.point_list:
                self.point_list.pop(0)  # Remove a primeira coordenada da lista de coordenadas
            self.condicao = True  # Define a condição como True para permitir a solicitação novamente
            sleep(3)  # Pausa a execução por 3 segundos
        print(f'Coordenada atual: {round(self.x, 2)} -- Faltando para chegar: {round(caminho_x, 2)}')  # Imprime a posição atual e a distância restante
        self.publisher.publish(msg)  # Publica o comando de velocidade


def main(args=None):
    rclpy.init(args=args)  # Inicializa o contexto do ROS 2
    turtle_controller = TurtleController()  # Cria uma instância do controlador da tartaruga
    rclpy.spin(turtle_controller)  # Mantém o programa em execução enquanto recebe mensagens
    turtle_controller.destroy_node()  # Encerra o nó do controlador da tartaruga
    rclpy.shutdown()  # Encerra o contexto do ROS 2

if __name__ == '__main__':
    main()

