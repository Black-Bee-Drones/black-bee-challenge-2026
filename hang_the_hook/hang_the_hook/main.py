import nectar
from nectar.vision.camera import ImageHandler
import line_follow
import checkpoint

nectar.init()

FRAME_WIDTH = 640

def process(frame):
    # linha
    cx, cy, angle = line_follow.segue_linha(frame)
    if cx is None:
        print("sem linha")
    else:
        desvio = cx - FRAME_WIDTH // 2
        if desvio > 15:
            print("Corrigir pra direita")
        elif desvio < -15:
            print("Corrigir pra esquerda")
        else:
            print("Reto.")
##Implemento de PID e controle de voo, alem de criação da classe


    # checkpoint
    passou, checkpoint_mask = checkpoint.detecta_checkpoint(frame)
    if passou:
        print("Checkpoint!")
    return frame

handler = ImageHandler(image_source="webcam", image_processing_callback=process, show_result="Camera")

handler.run()
nectar.spin()
nectar.shutdown()