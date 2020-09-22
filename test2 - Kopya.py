import numpy as np
import cv2

cap = cv2.VideoCapture('nvarguscamerasrc !  video/x-raw(memory:NVMM), width=3264, height=2464, \
format=NV12, framerate=21/1 \
! nvvidconv flip-method='+str(2)+' ! video/x-raw, width='+str(640)+', \
 height='+str(480)+', format=BGRx ! videoconvert ! \
video/x-raw, format=BGR ! appsink')

hata_kovaryansi = 1
kalman_katsayisi = 0.1
onceki_kalman = 0
kalman = []
dist = []

hata_kovaryansix = 1
kalman_katsayisix = 0.1
onceki_kalmanx = 0
kalmanx = []

hata_kovaryansiy = 1
kalman_katsayisiy = 0.1
onceki_kalmany = 0
kalmany = []



while True:
    ret, frame = cap.read()
    output = frame.copy()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    #Filtre ayarları
    gray = cv2.medianBlur(gray, 5)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    gray = cv2.bilateralFilter(gray, 5, 175, 175)

    gray = cv2.Canny(gray, 30, 30)


    if h1 >= 100:
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 35, param1=50, param2=h3, minRadius=50, maxRadius=250)
    else :
        h1 = 99
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 50, param1=35, param2=h3, minRadius=50, maxRadius=250)

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

            olculen_verix = x
            kalman_kazancix = hata_kovaryansix / (hata_kovaryansix + kalman_katsayisix)
            kalman_tahminx = onceki_kalmanx + (kalman_kazancix * (olculen_verix - onceki_kalmanx))
            hata_kovaryansix = (1 - kalman_kazancix) * hata_kovaryansix
            onceki_kalmanx = kalman_tahminx

            olculen_veriy = y
            kalman_kazanciy = hata_kovaryansiy / (hata_kovaryansiy + kalman_katsayisiy)
            kalman_tahminy = onceki_kalmany + (kalman_kazanciy * (olculen_veriy - onceki_kalmany))
            hata_kovaryansiy = (1 - kalman_kazanciy) * hata_kovaryansiy
            onceki_kalmany = kalman_tahminy

            kalman.append(kalman_tahmin)
            dist.append(distance)

            kalmanx.append(kalman_tahminx)
            kalmany.append(kalman_tahminy)

            if len(kalman) == 50:
                kalman.clear()
            if len(dist) == 50:
                dist.clear()

            if len(kalmanx) == 50:
                kalmanx.clear()
            if len(kalmany) == 50:
                kalmany.clear()
            a = 320
            b = 240

            cv2.circle(output, (a, b), 4,(0, 0, 255), -1)
            cv2.circle(output, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 0, 255), -1)
            cv2.line(output, (a, b), (x, y), (255, 255, 255), 2)
            cv2.imshow('HoughCircles', gray)
            xaxis = (x - a)
            yaxsis = (y- b )
            print(xaxis,yaxsis)

            text = "X = " + str(xaxis)
            text1 = "Y = " + str(yaxsis)
            text2 = "Radius = " + str(r)
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
            image=cv2.putText(output, stra, (0, 180), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0))
    cv2.imshow('Traking', output)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

plt.show()
cap.release()
cv2.destroyAllWindows()