String.prototype.format = function () {
    var i = 0, args = arguments;
    return this.replace(/{}/g, function () {
      return typeof args[i] != 'undefined' ? args[i++] : '';
    });
};

for(let i = 1; i <= 10; i++)
{
    d3.csv('./static/csv/track_{}.csv'.format(i), function(err, rows){

        function unpack(rows, key) {
            return rows.map(function(row)
            { return row[key]; }); }

        var gt = {
            name:'GT',
            x: unpack(rows, 'x1'),
            y: unpack(rows, 'y1'),
            z: unpack(rows, 'z1'),
            mode: 'lines',
            type: 'scatter3d',
            line: {
            
                color: 'rgb(0, 0, 0)',
                width: 4,
                dash: 'dot'
            },
        };

        var kf = {
            name:'I<sup>KF</sup>',
            x: unpack(rows, 'x2'),
            y: unpack(rows, 'y2'),
            z: unpack(rows, 'z2'),
            mode: 'lines',
            type: 'scatter3d',
            line: {
            
                color: '#1f77b4',
                width: 3
            },
        };

        var ekf = {
            name:'I<sup>EKF</sup>',
            x: unpack(rows, 'x3'),
            y: unpack(rows, 'y3'),
            z: unpack(rows, 'z3'),
            mode: 'lines',
            type: 'scatter3d',
            line: {
            
                color: 'firebrick',
                width: 3
            },
        };

        data = [gt, kf, ekf];
            
        Plotly.newPlot('track_{}'.format(i), data, {
            height: 640,
            scene : {
                aspectmode: "data",
           },
        });
      });
}
