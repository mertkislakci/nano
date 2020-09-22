import matplotlib.pyplot as plt
import numpy as np
import cv2

hata_kovaryansi = 1
kalman_katsayisi = 0.1
onceki_kalman = 0
kalman = []
dist = []

cap = cv2.VideoCapture("test1.mp4")
# Kamera ayarları standart 640x480
# cap.set(3,1280)
# cap.set(4,720)

while (True):
    ret, frame = cap.read()
    output = frame.copy()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    #Filtre ayarları
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    gray = cv2.medianBlur(gray, 5)
    gray = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, \
                                 cv2.THRESH_BINARY, 11, 3.5)
    #cv2.imshow('frame1', gray)

    #Matris ayarları
    kernel = np.ones((20, 20), np.uint64)
    gray = cv2.erode(gray, kernel, iterations=1)
    gray = cv2.dilate(gray, kernel, iterations=1)
    cv2.imshow("test",gray)

    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 250, param1=30, param2=45, minRadius=0, maxRadius=0)

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")

        for (x, y, r) in circles:

            area = (r**2)*3.14
            distance = 2 * (10 ** (-7)) * (area ** 2) - (0.0067 * area) + 83.487

            olculen_veri = distance
            kalman_kazanci = hata_kovaryansi / (hata_kovaryansi + kalman_katsayisi)
            kalman_tahmin = onceki_kalman + (kalman_kazanci * (olculen_veri - onceki_kalman))
            hata_kovaryansi = (1 - kalman_kazanci) * hata_kovaryansi
            onceki_kalman = kalman_tahmin

            kalman.append(kalman_tahmin)
            dist.append(distance)

            if len(kalman) == 50:
                kalman.clear()
            if len(dist) == 50:
                dist.clear()
            a = 320
            b = 240

            cv2.circle(output, (a, b), 4,(0, 0, 255), -1)
            cv2.circle(output, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 0, 255), -1)
            cv2.line(output, (a, b), (x, y), (255, 255, 255), 2)
        #cv2.imshow('gray', gray)
            xaxis = (x - a)
            yaxsis = (y- b )
            print(xaxis,yaxsis)

            text = "X=" + str(xaxis)
            text1 = "Y=" + str(yaxsis)
            text2 = "Radius=" + str(r)
            text3 = "Z= "+ str(int(distance))
            text4 = "Z Filtre = " + str(int(kalman_tahmin))
            font = cv2.FONT_HERSHEY_SIMPLEX
            org = (00, 30)
            org1 = (00, 60)
            org2 = (00, 90)
            org3 = (00, 120)
            org4 = (00, 150)
            fontScale = 1
            color = (0, 0, 255)
            thickness = 2

            image = cv2.putText(output, text, org, font, fontScale,
                                color, thickness, cv2.LINE_AA, False)
            image = cv2.putText(output, text1, org1, font, fontScale,
                                color, thickness, cv2.LINE_AA, False)
            image  = cv2.putText(output, text2, org3, font, fontScale,
                                color, thickness, cv2.LINE_AA, False)
            image = cv2.putText(output, text3, org2, font, fontScale,
                            color, thickness, cv2.LINE_AA, False)
            image = cv2.putText(output, text4, org4, font, fontScale,
                                color, thickness, cv2.LINE_AA, False)
    cv2.imshow('Traking', output)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
plt.figure("Veri Grafiği")
plt.plot(kalman,color="red")
plt.plot(dist,color="blue")
plt.figtext(.2, .0, "Mavi= Normal")
plt.figtext(.0, .0, "Kırmızı= Filtreli")
plt.show()
cap.release()
cv2.destroyAllWindows()