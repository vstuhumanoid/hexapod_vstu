window.onload = function () {
    var app = new Vue({
        el: ".container",
        data: {
            params: {
                stride_length: 0,
                step_height: 0,
                overshoot_support: 0,
                foot_body: 0,
                gait_type: 0,
                vx: 0,
                vy: 0,
                vz: 0,
                vux: 0,
                vuy: 0,
                vuz: 0
            },
            ros: null,
            frund_control: null
        },
        created: function() {
            const host = document.location.hostname;
            console.log('Hostname: ' + host);
            this.ros = new ROSLIB.Ros ({
                url: 'ws://'+document.location.hostname+':9090'
            });

            this.ros.on('connection', function() {
                    console.log('Connected to websocket server.');
                    this.frund_control = new ROSLIB.Topic({
                        ros : this.ros,
                        name : '/hexapod/walk_params/',
                        messageType : 'hexapod_msgs/FrundControl'
                    });
                }.bind(this));
            this.ros.on('error', function(error) {
                console.log('Error connecting to websocket server: ', error);
            });
            this.ros.on('close', function(error) {
                console.log('Connection to websocket server closed.');
            });
        },
        methods: {
            make_text: function() {
                let str = "";
                const values = Object.values(this.params);
                for (let i = 0; i < values.length; i++) {
                    str += values[i];
                    if (i != values.length - 1) {
                        str += ' ';
                    }
                }
                console.log(str);
            },
            send: function () {
                const message = new ROSLIB.Message ({
                    accel: {
                        linear : {
                            x : this.params.vx,
                            y : this.params.vy,
                            z : this.params.vz
                        },
                        angular : {
                            x : this.params.vux,
                            y : this.params.vuy,
                            z : this.params.vuz
                        }
                    },
                    gait_settings: {
                        step_length : this.params.stride_length,
                        step_height : this.params.step_height,
                        support_movement : this.params.overshoot_support,
                        foot_body_ratio : this.params.foot_body,
                        gait_type:  this.params.gait_type
                    }
                });
                this.frund_control.publish(message);
            }
        }
    })
}
