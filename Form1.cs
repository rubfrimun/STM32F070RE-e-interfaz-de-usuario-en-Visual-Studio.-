using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace trabajo_rfm
{
    public partial class Form1 : Form
    {
        double temperatura;
        double temperaRemoto;
        int velocidad_motor;
        int velocidaMotorRemoto;
        int pos_servo;
        int poServoRemoto;
        public Form1()
        {
            InitializeComponent();
            serialPort1.Encoding = System.Text.Encoding.GetEncoding("utf-8");
            //velocidadPP.Text = controlador_velPP.Value.ToString(); //debe ser la real del motor, que viene del micro
        }
        string texto = null;
        private void abrir_puerto_Click(object sender, EventArgs e)
        {
            string puertos = puerto.Text.ToString();
            serialPort1.PortName = puertos;
            serialPort1.Open();                         //no haria falta si ya se abre en el constructor?
            if (serialPort1.IsOpen)
            {
                label3.Text = "Puerto abierto";
                serialPort1.DiscardInBuffer();
            }
            else
            {
                label3.Text = "Puerto cerrado";
            }
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            //temperaturalcd.Value = temperatura;                   // si lo dejamos, sobreescribe cuando hay un 0
            realVelocidadPP.Text = Convert.ToString(velocidad_motor) + "    RPM";
            motor_velocimetro.Value = velocidad_motor;
            //textBoxCan.Text = texto;
            pos_actual.Text = Convert.ToString(pos_servo) + "    º";
            // valores en remoto
            veloMotoRemoto.Text = Convert.ToString(velocidaMotorRemoto) + "    RPM";
            posServoRemoto.Text = Convert.ToString(poServoRemoto) + "    º";


        }

        private void LED_ENVIO_Click(object sender, EventArgs e)
        {
            byte[] comando = new byte[4];
            int parpadeo = Convert.ToInt16(textBox2.Text);
            comando[0] = 0x4c;
            comando[1] = (byte)(parpadeo >> 8);
            comando[2] = (byte)(parpadeo);
            comando[3] = 0xE0;
            if (serialPort1.IsOpen)
            {
                serialPort1.Write(comando, 0, 4);
            }
        }

        private void serialPort1_DataReceived(object sender, System.IO.Ports.SerialDataReceivedEventArgs e)
        {
            byte[] comando = new byte[4];
            if (serialPort1.IsOpen)
            {
                while (serialPort1.BytesToRead >= 4)
                {
                    serialPort1.Read(comando, 0, 4);
                    switch (comando[0])
                    {
                        /*case 0x10:
                            if (comando[3] == 0xE0)
                            {
                                texto = "Contador = " + comando[1].ToString() + "\r\n" + texto;
                            }
                            else
                                texto = Encoding.Default.GetString(comando) + "\r\n" + texto;
                            break;
                        */
                        case 0x30:
                            if (comando[3] == 0xE0)
                            {
                                double decimalTEMP;
                                decimalTEMP = Convert.ToDouble(comando[2]);
                                temperatura = Convert.ToDouble(comando[1]) + decimalTEMP / 100;
                                temperaturalcd.Value = temperatura;
                            }
                            break;
                        case 0x40:
                            if (comando[3] == 0xE0)
                            {
                                velocidad_motor = comando[1];
                            }
                            break;
                        case 0x50:
                            if (comando[3] == 0xE0)
                            {
                                //double decimalDIST;
                                //decimalDIST = Convert.ToDouble(comando[2]);
                                distanciaLCD.Value = Convert.ToDouble(comando[1]); //+ decimalDIST / 100; no se reciben los decimales
                            }
                            break;
                        case 0x20:
                            if (comando[3] == 0xE0)
                            {
                                pos_servo = comando[1];
                            }
                            break;
                        case 0x19:
                            textStatusB.Invoke((MethodInvoker)delegate
                            {
                                // Running on the UI thread
                                textStatusB.Text = comando[1].ToString();
                            });
                            break;
                        /*case 0x21:
                            textRECB.Invoke((MethodInvoker)delegate
                            {
                                // Running on the UI thread
                                textRECB.Text = comando[1].ToString();
                                textTECB.Text = comando[2].ToString();
                            });
                            break;
                        case 0x22:
                            textStatusA.Invoke((MethodInvoker)delegate
                            {
                                // Running on the UI thread
                                textStatusA.Text = comando[1].ToString();
                            });
                            break;
                        
                        case 0x23:
                            textRECA.Invoke((MethodInvoker)delegate
                            {
                                // Running on the UI thread
                                textRECA.Text = comando[1].ToString();
                                textTECA.Text = comando[2].ToString();
                            });
                            break;                       
                            
                        case 0x15:
                            textBoxCan.Invoke((MethodInvoker)delegate
                            {
                                // Running on the UI thread
                                texto = "CAN A: Mensaje recibido: data6: " + comando[1].ToString() + " data7: " + comando[2].ToString() + "\r\n" + textBoxCan.Text;
                            });
                            break;*/
                        case 0x10:
                            textStatusA.Invoke((MethodInvoker)delegate
                            {
                                // Running on the UI thread
                                textStatusA.Text = comando[1].ToString();
                            });
                            break;
                        case 0x11:
                            textRECA.Invoke((MethodInvoker)delegate
                            {
                                // Running on the UI thread
                                textRECA.Text = comando[1].ToString();
                                textTECA.Text = comando[2].ToString();
                            });
                            break;                           
                        case 0x21:
                            if (comando[3] == 0xE0)
                            {
                                poServoRemoto = comando[1];
                            }
                            break;

                        case 0x31:
                            if (comando[3] == 0xE0)
                            {
                                double decimalTEMPremoto;
                                decimalTEMPremoto = Convert.ToDouble(comando[2]);
                                temperaRemoto = Convert.ToDouble(comando[1]) + decimalTEMPremoto / 100;
                                TempeRemoto.Value = temperaRemoto;
                            }
                            break;
                        case 0x41:
                            if (comando[3] == 0xE0)
                            {
                                velocidaMotorRemoto = comando[1];
                            }
                            break;
                        case 0x51:
                            if (comando[3] == 0xE0)
                            {
                                //double decimalDIST;
                                //decimalDIST = Convert.ToDouble(comando[2]);
                                DistanRemoto.Value = Convert.ToDouble(comando[1]); //+ decimalDIST / 100; no se reciben los decimales
                            }
                            break;
                        case 0x13:
                            textBoxCan.Invoke((MethodInvoker)delegate
                            {
                                textBoxCan.AppendText("Can A inicializado\r\n");
                            });
                            break;
                        case 0x14:
                            textBoxCan.Invoke((MethodInvoker)delegate
                            {
                                textBoxCan.AppendText("Can A error al inicializar\r\n");
                            });
                            break;
                        case 0x09:
                            textBoxCan.Invoke((MethodInvoker)delegate
                            {
                                textBoxCan.AppendText("Can A enviando información\r\n");
                            });
                        break;
                    }
                }

            }
            else
            {
                texto = "Error de apertura del puerto";
            }
        }

        private void button_tramaD_Click(object sender, EventArgs e)
        {
            byte[] trama = new byte[4];
            trama[0] = 0x44;                     // giro a derecha
            trama[1] = 0xFF;
            trama[2] = 0xFF;
            trama[3] = 0xE0;
            if (serialPort1.IsOpen)
            {
                serialPort1.Write(trama, 0, 4);  // envio de la trama
            }
        }

        private void button_tramaI_Click(object sender, EventArgs e)
        {
            byte[] trama = new byte[4];
            trama[0] = 0x49;                    // giro a izquierda
            trama[1] = 0xFF;
            trama[2] = 0xFF;
            trama[3] = 0xE0;
            if (serialPort1.IsOpen)
            {
                serialPort1.Write(trama, 0, 4);  // envio de la trama
            }
        }

        private void button_tama_C_Click(object sender, EventArgs e)
        {
            byte[] trama = new byte[4];
            trama[0] = 0x45;                    // detener giro
            trama[1] = 0xFF;                    // no tendria que mandar la velocidad, por si al iniciar le doy directamente a izq o der
            trama[2] = 0xFF;
            trama[3] = 0xE0;
            // velocidad_motor = 0;
            if (serialPort1.IsOpen)
            {
                serialPort1.Write(trama, 0, 4);  // envio de la trama
            }
        }

        private void controlador_velPP_KnobChangeValue(object sender, LBSoft.IndustrialCtrls.Knobs.LBKnobEventArgs e)
        {
            velocidadPP.Text = controlador_velPP.Value.ToString("F2");          // f2 para que tenga dos decimales
            byte[] trama = new byte[4];
            trama[0] = 0x4D;                    // control de velocidad
            trama[1] = (byte)(controlador_velPP.Value);
            trama[2] = 0xFF;
            trama[3] = 0xE0;
            if (serialPort1.IsOpen)
            {
                serialPort1.Write(trama, 0, 4);  // envio de la trama
            }


        }

        private void solicita_temp_Click(object sender, EventArgs e)
        {
            byte[] trama = new byte[4];
            trama[0] = 0x30;                     // solicitud de temperatura
            trama[1] = 0xFF;
            trama[2] = 0xFF;
            trama[3] = 0xE0;
            if (serialPort1.IsOpen)
            {
                serialPort1.Write(trama, 0, 4);     // envio de la trama
            }
        }

        private void frecuencia_Temp_Click(object sender, EventArgs e)
        {
            byte[] trama = new byte[4];
            int frec = Convert.ToInt16(frec_temp.Text);
            trama[0] = 0x54;                        // envio de frecuecncia
            trama[1] = (byte)(frec >> 8);    // manda el valor de la frecuencia con la que queremos que lea, aprovecha el envio 
            trama[2] = (byte)(frec);
            trama[3] = 0xE0;
            if (serialPort1.IsOpen)
            {
                serialPort1.Write(trama, 0, 4);     // envio de la trama
            }
        }

        private void solicita_tof_Click(object sender, EventArgs e)
        {
            byte[] trama = new byte[4];
            trama[0] = 0x60;                     // solicitud de distancia
            trama[1] = 0xFF;
            trama[2] = 0xFF;
            trama[3] = 0xE0;
            if (serialPort1.IsOpen)
            {
                serialPort1.Write(trama, 0, 4);     // envio de la trama
            }
        }

        private void stop_dist_Click(object sender, EventArgs e)
        {
            byte[] trama = new byte[4];
            trama[0] = 0x61;                     // detener lectura distancia
            trama[1] = 0xFF;
            trama[2] = 0xFF;
            trama[3] = 0xE0;
            if (serialPort1.IsOpen)
            {
                serialPort1.Write(trama, 0, 4);     // envio de la trama
            }
            distanciaLCD.Value = 0.0;
        }

        private void button1_Click(object sender, EventArgs e)
        {
            byte[] trama = new byte[4];
            trama[0] = 0x58;                     // solicitud de encender pantalla
            trama[1] = 0xFF;
            trama[2] = 0xFF;
            trama[3] = 0xE0;
            if (serialPort1.IsOpen)
            {
                serialPort1.Write(trama, 0, 4);     // envio de la trama
            }
        }

        private void all_of_Click(object sender, EventArgs e)
        {
            byte[] trama = new byte[4];
            trama[0] = 0x43;                     // detener todo
            trama[1] = 0xFF;
            trama[2] = 0xFF;
            trama[3] = 0xE0;
            //motor_velocimetro.Value = 0;
            velocidad_motor = 0;
            temperaturalcd.Value = 0.0;
            distanciaLCD.Value = 0.0;
            /* valores en remoto
            velocidaMotorRemoto = 0;
            temperaRemoto.Value = 0.0;
            DistanRemoto.Value = 0.0;*/
            textBoxCan.Text = string.Empty;
            textStatusA.Text = string.Empty;
            textTECA.Text = string.Empty;   
            textRECA.Text = string.Empty;

            if (serialPort1.IsOpen)
            {
                serialPort1.Write(trama, 0, 4);     // envio de la trama
            }
        }

        private void posServo_KnobChangeValue(object sender, LBSoft.IndustrialCtrls.Knobs.LBKnobEventArgs e)
        {
            byte[] trama = new byte[4];                         // envio de la  posicion del servo
            pos_servo_value.Text = posServo.Value.ToString("F2");
            trama[0] = 0x53;
            trama[1] = (byte)(posServo.Value);
            trama[2] = 0xFF;
            trama[3] = 0xE0;
            if (serialPort1.IsOpen)
            {
                serialPort1.Write(trama, 0, 4);     // envio de la trama
            }
        }

        private void servo_Click(object sender, EventArgs e)
        {
            byte[] trama = new byte[4];                         // activacion del servo
            int delay = Convert.ToInt16(espera.Text);
            int grados_segundo = Convert.ToInt16(velo_servo.Text);
            trama[0] = 0x41;
            trama[1] = (byte)(delay);
            trama[2] = (byte)(grados_segundo);
            trama[3] = 0xE0;
            if (serialPort1.IsOpen)
            {
                serialPort1.Write(trama, 0, 4);     // envio de la trama
            }
        }

        private void button2_Click(object sender, EventArgs e)
        {
            byte[] trama = new byte[4];
            trama[0] = 0x52;                     // solicitud de para recibir b y transmitir a
            trama[1] = 0xFF;
            trama[2] = 0xFF;
            trama[3] = 0xE0;
            if (serialPort1.IsOpen)
            {
                serialPort1.Write(trama, 0, 4);     // envio de la trama
            }
        }

        private void button1_Click_1(object sender, EventArgs e)
        {
            byte[] trama = new byte[4];
            trama[0] = 0x55;                     // solicitud de para solicitar estados
            trama[1] = 0xFF;
            trama[2] = 0xFF;
            trama[3] = 0xE0;
            if (serialPort1.IsOpen)
            {
                serialPort1.Write(trama, 0, 4);     // envio de la trama
            }
        }
    }
}