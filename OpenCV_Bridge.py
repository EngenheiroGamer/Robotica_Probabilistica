import numpy as np
import cv2 as cv

def main(cv_image):

    if cv_image is None:
        print("Erro: Imagem não recebida corretamente")
        return

    gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
    edged = cv.Canny(gray, 100, 300)

    contours, _ = cv.findContours(edged, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    square_detected = False

    for cnt in contours:
        approx = cv.approxPolyDP(cnt, 0.04 * cv.arcLength(cnt, True), True)

        if len(approx) == 4 and cv.isContourConvex(approx):
            area = cv.contourArea(approx)
            if area > 1000:  # Ajuste este valor conforme a escala da câmera
                square_detected = True
                cv.drawContours(cv_image, [approx], -1, (0, 255, 0), 3)

    if square_detected:
        print("Quadrado detectado na imagem!")
    else:
        print("Nenhum quadrado detectado.")

    cv.imshow("Detecção de Quadrado", cv_image)
    cv.waitKey(1)