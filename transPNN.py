import numpy as np
import math
import json
import urllib.request
import sys


def getArgs(argv):
    params = []
    for i in range(0, len(argv)):
        kv = argv[i].split(':')
        valu = kv[1].split('#')
        if(len(valu) > 1):
            values = []
            for v in valu:
                num = float(v)
                values.append(num)
            param = [kv[0], values]
            params.append(param)
        else:
            num = float(kv[1])
            params.append([kv[0], num])
    return dict(params)


class CameraMat(object):
    def __init__(self, ff=35, cameraAngle=[80, 50], imageSize=[640, 468], senseSize=[150, 86], position=[116.317317, 39.96533, 15], vectorCar=[0, 0, 1], vectorCdr=[0, 0.5, -1]):
        self.ff = ff
        self.imageSize = imageSize
        self.senseSize = senseSize
        self.cameraPosition = position
        self.vectCar = vectorCar
        self.vectCdr = vectorCdr
        self.cameraAngle = cameraAngle
        self.centerLon = 117.0
        self._level = 24
        self._high = 0

    def CalcInMatrix(self):
        imageSize = self.imageSize
        ff = self.ff
        senseSize = self.senseSize

        du = senseSize[0] / imageSize[0]
        dv = senseSize[1] / imageSize[1]
        fu = ff / du
        fv = ff / dv
        v0 = imageSize[1] / 2
        u0 = imageSize[0] / 2
        inMatrix = np.array([[fu, 0, u0], [0, fv, v0], [0, 0, 1]])
        return inMatrix

    def CalcRotationMatrix2(self):
        vectorBefor = self.vectCar
        vectorAfter = self.vectCdr
        crossProduct = [0 for col in range(3)]
        crossProduct[0] = vectorBefor[1] * \
            vectorAfter[2] - vectorBefor[2] * vectorAfter[1]
        crossProduct[1] = vectorBefor[2] * \
            vectorAfter[0] - vectorBefor[0] * vectorAfter[2]
        crossProduct[2] = vectorBefor[0] * \
            vectorAfter[1] - vectorBefor[1] * vectorAfter[0]
        u = np.array(crossProduct)

        dotProduct = np.dot(vectorBefor, vectorAfter)
        nomalBefor = math.sqrt(np.dot(vectorBefor, vectorBefor))
        nomalAfter = math.sqrt(np.dot(vectorAfter, vectorAfter))

        rotationAngle = math.acos(dotProduct / nomalBefor / nomalAfter)
        u = u / math.sqrt(np.dot(u, u))

        rotatinMatrix = [[0 for col in range(3)] for row in range(3)]
        rotatinMatrix[0][0] = math.cos(
            rotationAngle) + u[0] * u[0] * (1 - math.cos(rotationAngle))
        rotatinMatrix[1][0] = u[0] * u[1] * \
            (1 - math.cos(rotationAngle) - u[2] * math.sin(rotationAngle))
        rotatinMatrix[2][0] = u[1] * \
            math.sin(rotationAngle) + u[0] * \
            u[2] * (1 - math.cos(rotationAngle))

        rotatinMatrix[0][1] = u[2] * \
            math.sin(rotationAngle) + u[0] * \
            u[1] * (1 - math.cos(rotationAngle))
        rotatinMatrix[1][1] = math.cos(
            rotationAngle) + u[1] * u[1] * (1 - math.cos(rotationAngle))
        rotatinMatrix[2][1] = -u[0] * \
            math.sin(rotationAngle) + u[1] * \
            u[2] * (1 - math.cos(rotationAngle))

        rotatinMatrix[0][2] = -u[1] * \
            math.sin(rotationAngle) + u[0] * \
            u[2] * (1 - math.cos(rotationAngle))
        rotatinMatrix[1][2] = u[0] * \
            math.sin(rotationAngle) + u[1] * \
            u[2] * (1 - math.cos(rotationAngle))
        rotatinMatrix[2][2] = math.cos(
            rotationAngle) + u[2] * u[2] * (1 - math.cos(rotationAngle))

        rotatinMatrixEx = np.array(rotatinMatrix)
        return rotatinMatrixEx

    def OuterMatrix(self):
        cameraH = self.cameraPosition[2]

        ''' pi = math.pi
        rotationAngle = np.array([50 * pi / 180, 50 * pi / 180, 50 * pi / 180])
        rotationMatrix = CalcRotationMatrix1(rotationAngle) '''

        rotationMatrix = self.CalcRotationMatrix2()
        # test = np.dot(cdrVect, np.linalg.inv(rotationMatrix))
        translationVector = np.array([0, 0, cameraH])
        # t1 = np.row_stack((rotationMatrix, np.array([0, 0, 0])))

        outerMatrix = np.column_stack(
            (rotationMatrix, np.transpose(translationVector)))
        return outerMatrix

    def GetPosition(self, geoPosition):
        centerLon = self.centerLon
        cameraPosition = self.cameraPosition
        relatPoint = ChangeXYZ(centerLon, cameraPosition, geoPosition)

        inM = self.CalcInMatrix()

        # inMEx = np.column_stack((inM, np.transpose(np.array([0, 0, 0]))))

        outM = self.OuterMatrix()
        a = outM[:, 3]
        b = np.linalg.inv(outM[:, 0:3])

        # outMI = np.column_stack((b, np.transpose(a)))
        outMII = np.column_stack((np.transpose(b),  np.transpose(a)))
        # outMII = np.transpose(b)
        # testVect = [0, 1.5, 2, 1]
        # retest3 = np.dot(np.transpose(b), np.transpose(testVect))
        # retest = np.dot(outMII, np.transpose(testVect))
        # retest2 = np.dot(testVect, outMI)
        position = np.transpose(
            np.array([relatPoint[0], relatPoint[1], geoPosition[2], 1]))
        res = np.dot(inM, np.dot(outMII, position))
        return res

    def GetRange(self):
        carVect = self.vectCar
        cdrVect = self.vectCdr

        outM = self.OuterMatrix()

        outMI = outM[:, 0:3]
        # outMI = np.linalg.inv(outM[:, 0:3])
        cdrVect = np.dot(outMI, np.transpose(carVect))
        cameraAngLon = self.cameraAngle[0]
        cameraAngLat = self.cameraAngle[1]
        cameraH = self.cameraPosition[2]
        ag = CalcVectorAngle(carVect, cdrVect)
        if(abs(ag) > math.pi / 2):
            angele = math.pi - abs(ag)
        else:
            angele = abs(ag)
        # rotationMatrix = CalcRotationMatrix2(carVect, cdrVect)
        # minL = cameraH / math.cos(angele - Radians(cameraAngLat / 2))
        maxL = cameraH / math.cos(angele + Radians(cameraAngLat / 2))
        minL = cameraH / math.cos(angele - Radians(cameraAngLat / 2))
        maxLenth = maxL * math.tan(Radians(cameraAngLon / 2)) * 2 * 2.5
        minLenth = minL * math.tan(Radians(cameraAngLon / 2)) * 2

        maxR = cameraH * math.tan(angele + Radians(cameraAngLat / 2)) * 3
        minR = cameraH * math.tan(angele - Radians(cameraAngLat / 2))
        # wighth = maxR - minR
        if(cdrVect[0] != 0):
            sita = math.atan(cdrVect[1] / cdrVect[0])
            if(cdrVect[1] < 0 and cdrVect[0] > 0):
                sita = math.pi + sita
            if(cdrVect[1] < 0 and cdrVect[0] < 0):
                sita = sita - math.pi
        elif(cdrVect[1] > 0):
            sita = math.pi / 2
        elif(cdrVect[1] < 0):
            sita = math.pi * 3 / 2
        rotationMatrix2 = np.array(
            [[math.cos(sita), -math.sin(sita), 0], [math.sin(sita), math.cos(sita), 0], [0, 0, 1]])
        # rotationMatrix2 = np.array(
        #   [[-math.sin(sita), math.cos(sita), 0], [math.cos(sita), math.sin(sita), 0], [0, 0, 1]])

        po1 = np.array([minR, -minLenth / 2, 1])
        po2 = np.array([minR, minLenth / 2, 1])
        po3 = np.array([maxR, maxLenth / 2, 1])
        po4 = np.array([maxR, -maxLenth / 2, 1])

        po1n = np.dot(rotationMatrix2, po1)
        po2n = np.dot(rotationMatrix2, po2)
        po3n = np.dot(rotationMatrix2, po3)
        po4n = np.dot(rotationMatrix2, po4)

        return(Point2Geo(self.cameraPosition, po1n, po2n, po3n, po4n))

    def GenCameraJson(self):
        pos = self.GetRange()
        minLon = pos[0][0]
        minLat = pos[0][1]
        maxLon = pos[0][0]
        maxLat = pos[0][1]
        for po in pos:
            if po[0] < minLon:
                minLon = po[0]
            if po[0] > maxLon:
                maxLon = po[0]
            if po[1] < minLat:
                minLat = po[1]
            if po[1] > maxLat:
                maxLat = po[1]
        po1 = [minLon, minLat]
        po2 = [maxLon, maxLat]
        # po1 = [116.317416, 39.965014]
        # po2 = [116.31843, 39.966012]
        level = self._level
        high = self._high
        url = GenUrl(po1, po2, level)
        print(url)
        geoPoints = json.load(urllib.request.urlopen(url))
        # rangeJson = []
        envelops = geoPoints['envelops']
        cameraJson = []
        for item in envelops:
            code = item['code']
            rg = item['range']
            rectangle = [[rg['minLon'], rg['minLat'], high], [rg['minLon'], rg['maxLat'], high], [
                rg['maxLon'], rg['maxLat'], high],  [rg['maxLon'], rg['minLat'], high]]

            rec = []
            for po in rectangle:
                po22 = self.GetPosition(po)
                po22 = po22 / po22[2]
                rec.append(list(np.round(po22)))

            recJson = {
                'code': code,
                'range': rec
            }
            print(recJson)
            cameraJson.append(recJson)

        # recJsons = json.dumps(cameraPosition)
        with open('rangeJson.json', 'w') as f:
            json.dump(cameraJson, f)
        return cameraJson


def ChangeXYZ(centerLon, cameraPosition, geoPosition):
    cameraPo = LonLat2UTM(centerLon, [cameraPosition[0], cameraPosition[1]])
    pointPo = LonLat2UTM(centerLon, [geoPosition[0], geoPosition[1]])

    ppo = [pointPo[0] - cameraPo[0], pointPo[1] -
           cameraPo[1], cameraPosition[2]]

    return ppo


def LonLat2UTM(centerLon, position):
    c, e, f = centerLon, position[1], position[0]
    d = math.floor(c) + (math.floor(c * 100) - math.floor(c) * 100) / \
        60 + (c * 10000 - math.floor(c * 100) * 100) / 3600
    g = math.floor(e) + (math.floor(e * 100) - math.floor(e) * 100) / \
        60 + (e * 10000 - math.floor(e * 100) * 100) / 3600
    h = math.floor(f) + (math.floor(f * 100) - math.floor(f) * 100) / \
        60 + (f * 10000 - math.floor(f * 100) * 100) / 3600
    i = h - d
    j = i / 57.2957795130823
    k = math.tan(Radians(g))
    l = math.cos(Radians(g))
    m = 0.006738525415 * l * l
    n = k * k
    o = 1 + m
    p = 6399698.9018 / math.sqrt(o)
    q = j * j * l * l
    r = k * l
    s = r * r
    t = (32005.78006 + s * (133.92133 + s * 0.7031))
    utmX = 6367558.49686 * g / 57.29577951308 - r * l * t + \
        ((((n - 58) * n + 61) * q / 30 + (4 * m + 5) * o - n)
         * q / 12 + 1) * p * k * q / 2
    utmY = ((((n - 18) * n - (58 * n - 14) * m + 5) *
             q / 20 + o - n) * q / 6 + 1) * p * (j * l)
    return [utmY, utmX]


def Radians(angle):
    return angle * math.pi / 180


def GenUrl(po1, po2, level):
    url = "http://47.92.3.2:800/geoSOT-API/Range2Envelops/{1}/{0}/{3}/{2}/{4}".format(
        po1[1], po1[0], po2[1], po2[0], level)
    return url


def Point2Geo(cameraPosition, *vects):
    res = []
    if(vects):
        for vect in vects:
            xGeo = vect[0] / 111111 + cameraPosition[0]
            yGeo = vect[1] / 111111 * \
                math.cos(cameraPosition[1]) + cameraPosition[1]
            res.append([xGeo, yGeo])
    return res


def CalcVectorAngle(ve1, ve2):
    return math.acos(np.dot(ve1, ve2) / (np.linalg.norm(ve1) * np.linalg.norm(ve2)))


if(__name__ == '__main__'):
    argv = sys.argv[1:]
    args = getArgs(argv)
    print(args)
    keymap = ['ff', 'cameraAngle', 'imageSize',
              'senseSize', 'position', 'vectorCar', 'vectorCdr']
    '''
    for item in list(args):
        if(item not in keymap):
            args.pop(item)
    '''
    # A simple method to genNew params

    args = {k: v for k, v in args.items() if k in keymap}

    cMat = CameraMat(**args)
    cMat.GenCameraJson()
