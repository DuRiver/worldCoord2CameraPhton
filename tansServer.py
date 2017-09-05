
from flask import Flask, abort, jsonify
from flask import request
from flask import make_response
import transPC

app = Flask(__name__)

defult = [
    {
        'title': u'wrong calculate',
        'description': u'check agian and try',
        'done': False
    }
]


@app.errorhandler(400)
def not_found(error):
    return make_response(jsonify({'error': 'Bad request'}), 400)


@app.errorhandler(404)
def not_found(error):
    return make_response(jsonify({'error': 'Not found'}), 404)


@app.route('/transP/api/v1.0/params/', methods=['GET'])
def get_task():
    # task = list(filter(lambda t: t['id'] == param_id, tasks))
    paramsDict = request.args.to_dict()
    parames = dealDictParam(paramsDict)

    
    cmat = transPC.CameraMat(
        parames[0], parames[1], parames[2], parames[3], parames[4], parames[5])
    jsonP = cmat.GenCameraJson()
    if len(jsonP) == 0:
        abort(404)
    # return jsonify({'result': task})
    return 'callback(' + str(jsonP) + ')'


def dealDictParam(dictP):
    dictP = dict(dictP)
    values = dictP.values()
    lenth = len(values)
    ff = dictP.get('ff', values[0])
    cameraAngle1 = dictP.get('cameraAngle1', values[1])
    cameraAngle2 = dictP.get('cameraAngle2', values[2])
    imageSize1 = dictP.get('imageSize1', values[3])
    imageSize2 = dictP.get('imageSize2', values[4])
    senseSize1 = dictP.get('senseSize1', values[5])
    senseSize2 = dictP.get('senseSize2', values[6])
    position1 = dictP.get('position1', values[7])
    position2 = dictP.get('position2', values[8])
    position3 = dictP.get('position3', values[9])
    vectorCdr1 = dictP.get('vectorCdr1', values[10])
    vectorCdr2 = dictP.get('vectorCdr2', values[11])
    vectorCdr3 = dictP.get('vectorCdr3', values[12])
    cameraAngle = [cameraAngle1, cameraAngle2]
    imageSize = [imageSize1, imageSize2]
    senseSize = [senseSize1, senseSize2]
    position = [position1, position2, position3]
    vectorCdr = [vectorCdr1, vectorCdr2, vectorCdr3]
    params = [ff, cameraAngle, imageSize, senseSize, position, vectorCdr]
    return params


def main():
    app.run(host='0.0.0.0', port=5053, debug=True)


if __name__ == '__main__':
    main()
