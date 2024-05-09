# cv2 numpy
import cv2
import numpy as np

# custom
from logger.logger import log

global drawing, last_point, background, canvas, output_background, output_canvas


def sortValues(numbers):
    indexed_numbers = list(enumerate(numbers))
    sorted_indices = [
        index
        for index, value in sorted(indexed_numbers, key=lambda x: x[1], reverse=True)
    ]
    return sorted_indices


def drawInit():
    global drawing, last_point, background, canvas, output_background, output_canvas
    background = (500, 500, 1)
    canvas = np.ones(background, dtype=np.uint8)
    drawing = False
    last_point = (0, 0)

    output_background = (500, 150, 1)
    output_canvas = np.ones(output_background, dtype=np.uint8) * 255
    cv2.putText(
        output_canvas, "Output", (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2
    )
    cv2.namedWindow("Draw a Number")
    # callback
    cv2.setMouseCallback("Draw a Number", drawNumber)


def clear():
    global background, canvas, output_background, output_canvas
    canvas = np.ones(background, dtype=np.uint8)
    output_canvas = np.ones(output_background, dtype=np.uint8) * 255


def drawNumber(event, x, y, flags, param):
    global drawing, last_point, background, canvas
    draw_color = (255, 255, 255)

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        last_point = (x, y)
    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            cv2.line(canvas, last_point, (x, y), draw_color, 30)
            last_point = (x, y)
    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False


def drawOutput(outputs, decision_values):
    global output_canvas
    for i in range(10):
        cv2.putText(
            output_canvas,
            f"{outputs[i]}:{decision_values[outputs[i]]:.2f}",
            (0, 50 + i * 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 0, 0),
            2,
        )


def loopEvent(call_f):
    global canvas, output_canvas, background, output_background
    log("Initing...", "DEBUG")
    drawInit()
    log("Loop user event", "DEBUG")
    while True:
        cv2.imshow("Draw a Number", cv2.hconcat([canvas, output_canvas]))
        key = cv2.waitKey(1) & 0xFF
        if key == ord("c"):
            log("The user presses C to clear the canvas", "DEBUG")
            clear()
        elif key == ord("s"):
            log("The user presses S to callback function", "DEBUG")
            decision_values = call_f(input=canvas)
            outputs = sortValues(decision_values)
            log(f"The outputs are {decision_values}", "DEBUG")
            log(f"{outputs}", "DEBUG")
            # clear
            clear()
            drawOutput(outputs, decision_values)
        elif key == ord("q"):
            log("Quit", "DEBUG")
            break
    cv2.destroyAllWindows()
