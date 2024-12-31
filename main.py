import matplotlib.pyplot as plt

from util import ReadData, ControlData

from coppeliasim import Coppeliasim

from flask import Flask, request
import threading

main = None
app = Flask(__name__)


#
# A class for the entire pick-and-place operation
#
class MainClass:
    def __init__(self):
        self.sim = Coppeliasim()
        self.visual_objects = [None] * 100

    def callback(self, read_data: ReadData, control_data: ControlData):
        pass
        # self.visualize(read_data.img)

    def visualize(self, img):
        # remove visual objects
        for i in range(len(self.visual_objects)):
            if self.visual_objects[i] is None:
                break
            self.visual_objects[i].remove()
            self.visual_objects[i] = None
        # display image
        self.visual_objects[0] = plt.imshow(img)
        plt.pause(0.001)

    def run(self):
        self.sim.run(self.callback)


@app.route("/stop")
def stop():
    main.sim.run_flag = False
    return


@app.route("/mission", methods=["POST"])
def mission():
    params = request.get_json()
    return ""


if __name__ == "__main__":
    main = MainClass()

    # start flask web server
    threading.Thread(
        target=lambda: app.run(
            host="0.0.0.0", port=5555, debug=False, use_reloader=False
        ),
        daemon=True,
    ).start()

    # run
    main.run()
