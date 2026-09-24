from nectar import(
    init as nectar_init,
    shutdown as nectar_shutdown,
)
from nectar.vision.camera import ImageHandler
from hang_the_hook.utils.line_follow import segue_linha
from rclpy import(
    init as rclpy_init,
    spin as rclpy_spin,
    shutdown as rclpy_shutdown,
)
from rclpy.node import Node

class Line(Node):
    def __init__(self):
        super().__init__('line')

        self.FRAME_WIDTH = 640
        self.desvio = 0

        # Passa o self.process como callback
        self.handler = ImageHandler(
            image_source="webcam",
            image_processing_callback=self.process,
            show_result="Camera"
        )
        self.handler.run()

    def process(self, frame):
        # TODO: Lógica do line_follow e controle de PID
         # linha

        cx, cy, angle = segue_linha(frame)

        if cx is None:

            print("sem linha")

        else:

            self.desvio = cx - self.FRAME_WIDTH // 2

        if self.desvio > 15:

            print("Corrigir pra direita")

        elif self.desvio < -15:

            print("Corrigir pra esquerda")

        else:

            print("Reto.")

        ##Implemento de PID e controle de voo, alem de criação da classe


def main(args=None):
    rclpy_init(args=args)
    nectar_init()

    node = Line()

    try:
        rclpy_spin(node)
    except KeyboardInterrupt:
        print("Parando...")
    finally:
        nectar_shutdown()
        node.destroy_node()
        rclpy_shutdown()

if __name__ == '__main__':
    print(f"\033[{10}F\033[J", end="", flush=True)
    main()
