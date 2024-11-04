# kit_motor_driver

Esse é o pacote ROS do Kit de Robótica dedicado ao controle de motores.

## Dependencias

Clone esse pacote dentro do seu ROS workspace, dentro da pasta src. Para isso, substitua o endereço e nome do seu ROS workspace e copie os comandos abaixo:
```bash
cd ~/ros2_ws/src/
git clone git@github.com:pedro-fuoco/kit_motor_driver.git
```

### Instalação manual de bibliotecas
Infelizmente, uma das bibliotecas utilizadas nesse pacote não está nos repositorios de indice do rosdep. Por conta disso, precisamos instala-la manualmente seguindo o seguinte comando:

```bash
sudo apt install python3-rpi-lgpio
```

### Colcon
Inicialize o seu ROS workspace e compile ele utilizando a ferramenta `colcon`. Mais uma vez, é necessario substituir o endereço e nome do seu próprio ROS workspace abaixo:
```bash
cd ~/ros2_ws/
source install/setup.sh
colcon build --packages-select kit_motor_driver
```
Fique de olho em possiveis erros nesse processo e realize debug se necessario.

## Configurações
As configurações desse pacote são feitas através de ROS `params`. Os arquivos que descrevem as configurações, que podem ser editados manualmente, se encontram na pasta `config`.
Lembre-se sempre de recompilar o pacote depois de fazer mudanças na configuração, com:
```bash
cd ~/ros2_ws/
source install/setup.sh
colcon build --packages-select kit_motor_driver
```

## Launch
Para iniciar o programa `motor_node`, responsável por externalizar para o ROS o controle dos motores do Kit, basta utilizar o seguinte comando:
```bash
ros2 launch kit_motor_driver motor_launch.py
```

Por convenção, valores positivos de duty cycle em ambas as rodas fazem o robô se mover na direção X positiva (frente do robô), enquanto valores negativos tem o resultado oposto.