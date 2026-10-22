{
  "startPoint": {
    "x": 91.45803922625564,
    "y": 132.80886824253122,
    "name": "Start",
    "locked": false,
    "headingDeg": 90
  },
  "lines": [
    {
      "id": "line-mutvh1ux-c2gaf9",
      "color": "#79D86B",
      "name": "IntakeCorner",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 133.16737965735635,
        "y": 133.58801303475803
      },
      "controlPoints": [],
      "heading": {
        "type": "constant",
        "reverse": false,
        "degrees": 0
      }
    },
    {
      "id": "line-mutvj04j-9yeb1d",
      "color": "#8CBCB7",
      "name": "TravelThroughHive",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 80.9863350625012,
        "y": 11.428506823111011
      },
      "controlPoints": [
        {
          "x": 69.478898007034,
          "y": 140.12075029308326
        },
        {
          "x": 85.32239155920283,
          "y": 61.350527549824136
        }
      ],
      "heading": {
        "type": "piecewise",
        "reverse": true,
        "piecewiseHeading": {
          "segments": [
            {
              "startProgress": 0,
              "endProgress": 0.11660214206231453,
              "interpolationType": "constant",
              "reversed": false,
              "continueFromPrevious": false,
              "parameters": {
                "degrees": 0
              }
            },
            {
              "startProgress": 0.11660214206231453,
              "endProgress": 0.7804096346439169,
              "interpolationType": "tangential",
              "reversed": true,
              "continueFromPrevious": false
            },
            {
              "startProgress": 0.7804096346439169,
              "endProgress": 1,
              "interpolationType": "linear",
              "reversed": false,
              "continueFromPrevious": true,
              "parameters": {
                "startDeg": 0,
                "endDeg": 180
              }
            }
          ]
        },
        "degrees": 0,
        "startDeg": 0,
        "endDeg": 180
      }
    },
    {
      "id": "line-mutvr42o-czfmlr",
      "color": "#CC5A85",
      "name": "RunToFlower",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 1000,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 93.69547680783636,
        "y": 14.589181791990793
      },
      "controlPoints": [
        {
          "x": 105.0052754982415,
          "y": 59.2546801729191
        }
      ],
      "heading": {
        "type": "linear",
        "reverse": true,
        "startDeg": 180,
        "endDeg": 270
      }
    },
    {
      "id": "line-mutvsl91-mojaf9",
      "color": "#99BDC6",
      "name": "ReturnFromFlower",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 80.54054362926941,
        "y": 11.38812089261617
      },
      "controlPoints": [
        {
          "x": 104.4489014749563,
          "y": 60.1466350726313
        }
      ],
      "heading": {
        "type": "linear",
        "reverse": true,
        "startDeg": 270,
        "endDeg": 180
      }
    },
    {
      "id": "line-mutvtkyo-sqgo9x",
      "color": "#9C78AC",
      "name": "Park",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 127.24379462321029,
        "y": 21.271633055398222
      },
      "controlPoints": [
        {
          "x": 93.62724306688419,
          "y": 38.141109298531816
        }
      ],
      "heading": {
        "type": "constant",
        "reverse": true,
        "degrees": 180
      }
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 141.5,
          "y": 70
        },
        {
          "x": 141.5,
          "y": 141.5
        },
        {
          "x": 118.3,
          "y": 141.5
        },
        {
          "x": 135.5,
          "y": 118
        },
        {
          "x": 136.3,
          "y": 70.2
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 6.2,
          "y": 116.9
        },
        {
          "x": 25,
          "y": 141.5
        },
        {
          "x": 0,
          "y": 141.5
        },
        {
          "x": 0,
          "y": 70
        },
        {
          "x": 6,
          "y": 70
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-mutvh1ux-c2gaf9"
    },
    {
      "kind": "path",
      "lineId": "line-mutvj04j-9yeb1d"
    },
    {
      "kind": "path",
      "lineId": "line-mutvr42o-czfmlr"
    },
    {
      "kind": "path",
      "lineId": "line-mutvsl91-mojaf9"
    },
    {
      "kind": "path",
      "lineId": "line-mutvtkyo-sqgo9x"
    }
  ],
  "fieldPoints": [],
  "version": "1.5.0",
  "timestamp": "2026-10-22T02:53:59.093Z"
}