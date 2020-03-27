﻿
/*********************************************************************/

// Created by Judith Amores

/*********************************************************************/

/* This class connects to Ezzence via Bluetooth and sends data out to the phone.
   It automatically scans and connects to Ezzence and also receives notification data from the device.
   Base BLE connection modified from Shatalmic package (Bluetooth LE for iOS, tvOS and Android).
   The Scene I created has been tested in Mac and iOS and successfully connects to the Ezzence device.

/* Info about the Ezzence BLE:
   Base UUID: 6E400001-B5A3-F393-­E0A9-­E50E24DCCA9E
   Basic UART connection over two lines, TXD and RXD. Based on a the UART service from Adafruit, which is a proprietary UART service specification by Nordic Semiconductors. 
   Data sent to and from this BLE can be viewed using the nRFUART apps from Nordic Semiconductors as well as the Bluefruit Connect app for iOS/Android.
   The Ezzence BLE is based on: https://learn.adafruit.com/introducing-adafruit-ble-bluetooth-low-energy-friend/uart-service
*/

using UnityEngine;
using System.Collections.Generic;
using UnityEngine.UI;
using System.Text;

public class EzzenceBLEController : MonoBehaviour
{
	public string DeviceName = "Ezzence";
	public string ServiceUUID = "6E400001-B5A3-F393-­E0A9-­E50E24DCCA9E"; //Base UUID 0001
	public string Characteristic = "6E400002-B5A3-F393-­E0A9-­E50E24DCCA9E"; //TX UUID 0x0002 (Write)
	public string CharacteristicRX = "6E400003-B5A3-F393-­E0A9-­E50E24DCCA9E"; //RX UUID 0x003 (Read)

	public Text Ezzence_Status;
	public Text BluetoothStatus;
	public GameObject PanelMiddle;
	public Text TextToSend;

	enum States
	{
		None,
		Scan,
		Connect,
		Subscribe,
		Unsubscribe,
		Disconnect,
		Communication,
	}

	private bool _workingFoundDevice = true;
	private bool _connected = false;
	private float _timeout = 0f;
	private States _state = States.None;
	private bool _foundID = false;

	// this is our Ezzence device
	private string _ezzence;

	public void OnButton(Button button)
	{
		if (button.name.Contains ("Send"))
		{
			if (string.IsNullOrEmpty (TextToSend.text))
			{
				BluetoothStatus.text = "Enter text to send...";
			}
			else
			{
				SendString (TextToSend.text);
			}
		}
		if (button.name.Contains("Toggle"))
		{
			//SendByte (0x01);
			SendString ("!B516");
		}
		if (button.name.Contains("Burst"))
		{
			//SendByte (0x01);
			SendString ("!B516");
		}
	}

	void Reset ()
	{
		_workingFoundDevice = false;	// used to guard against trying to connect to a second device while still connecting to the first
		_connected = false;
		_timeout = 0f;
		_state = States.None;
		_foundID = false;
		_ezzence = null;
		PanelMiddle.SetActive (false);
	}

	void SetState (States newState, float timeout)
	{
		_state = newState;
		_timeout = timeout;
	}

	void StartProcess ()
	{
		BluetoothStatus.text = "Initializing...";

		Reset ();
		BluetoothLEHardwareInterface.Initialize (true, false, () => {
			
			SetState (States.Scan, 0.1f);
			BluetoothStatus.text = "Initialized";

		}, (error) => {
			
			BluetoothLEHardwareInterface.Log ("Error: " + error);
		});
	}

	// Use this for initialization
	void Start ()
	{
		Ezzence_Status.text = "";

		StartProcess ();
	}
	
	// Update is called once per frame
	void Update ()
	{
		if (_timeout > 0f)
		{
			_timeout -= Time.deltaTime;
			if (_timeout <= 0f)
			{
				_timeout = 0f;

				switch (_state)
				{
				case States.None:
					break;

				case States.Scan:
					BluetoothStatus.text = "Looking for the Ezzence device...";

					BluetoothLEHardwareInterface.ScanForPeripheralsWithServices (null, (address, name) => {

						// we only want to look at devices that have the name we are looking for
						// this is the best way to filter out devices
						if (name.Contains (DeviceName))
						{
							_workingFoundDevice = true;

							// it is always a good idea to stop scanning while you connect to a device
							// and get things set up
							BluetoothLEHardwareInterface.StopScan ();
							BluetoothStatus.text = "";

							// add it to the list and set to connect to it
							_ezzence = address;

							Ezzence_Status.text = "Found Ezzence! Address: "+address;

							SetState (States.Connect, 3f);

							_workingFoundDevice = false;
						}

					}, null, false, false);
					break;

				case States.Connect:

					_foundID = false;

					Ezzence_Status.text = "Connecting to Ezzence";

					BluetoothLEHardwareInterface.ConnectToPeripheral (_ezzence, null, null, (address, serviceUUID, characteristicUUID) => {
						BluetoothStatus.text = serviceUUID +","+ ServiceUUID;

						if (IsEqual (serviceUUID, ServiceUUID))
						{
							BluetoothStatus.text = "Found service. Set the state.";
							Ezzence_Status.text = "Found service. Set the state.";

							// if we have found the characteristic that we are waiting for
							// set the state. make sure there is enough timeout that if the
							// device is still enumerating other characteristics it finishes
							// before we try to subscribe

							if (IsEqual (characteristicUUID, Characteristic))
							{
								BluetoothStatus.text = "Found characteristic. CONNECTED!";
								Ezzence_Status.text = "Found characteristic. CONNECTED!.";
								_connected = true;
								SetState (States.Subscribe, 2f);
							}
						}
					}, (disconnectedAddress) => {
						BluetoothLEHardwareInterface.Log ("Device disconnected: " + disconnectedAddress);
						Ezzence_Status.text = "Disconnected";
					});
					break;

				case States.Subscribe:
					BluetoothStatus.text = "Subscribing to Ezzence";
					Ezzence_Status.text = "Subscribing to Ezzence";

					BluetoothLEHardwareInterface.SubscribeCharacteristicWithDeviceAddress (_ezzence, ServiceUUID, Characteristic, null, (address, characteristicUUID, bytes) => {
						BluetoothStatus.text = "Received Serial: " + Encoding.UTF8.GetString (bytes);
						Ezzence_Status.text = "Received Serial: " + Encoding.UTF8.GetString (bytes);
					});

					// set to the none state and the user can start sending and receiving data
					_state = States.None;
					Ezzence_Status.text = "Waiting...";
					BluetoothStatus.text = "Start sending and receiving data...";

					PanelMiddle.SetActive (true);
					break;

				/*case States.Unsubscribe:
					BluetoothLEHardwareInterface.UnSubscribeCharacteristic (_ezzence, ServiceUUID, Characteristic, null);
					SetState (States.Disconnect, 4f);
					break;

				case States.Disconnect:
					if (_connected)
					{
						BluetoothLEHardwareInterface.DisconnectPeripheral (_ezzence, (address) => {
							BluetoothLEHardwareInterface.DeInitialize (() => {
								
								_connected = false;
								_state = States.None;
								Ezzence_Status.text = "Disconnected";
							});
						});
					}
					else
					{
						BluetoothLEHardwareInterface.DeInitialize (() => {
							
							_state = States.None;
						});
					}
					break;*/
				}
			}
		}
	}

	string FullUUID (string uuid)
	{
		return "0001" + uuid + "-B5A3-F393-E0A9-E50E24DCCA9E";
	}
	
	bool IsEqual(string uuid1, string uuid2)
	{
		if (uuid1.Length == 4)
			uuid1 = FullUUID (uuid1);
		if (uuid2.Length == 4)
			uuid2 = FullUUID (uuid2);

		return (uuid1.ToUpper().Equals(uuid2.ToUpper()));
	}

	void SendString(string value)
	{
		var data = Encoding.UTF8.GetBytes (value);
		// notice that the 6th parameter is set to "true". this can also be false depending if the device supports withResponse writing to its characteristic.
		// some devices do support this setting and it is prefered when they do so that you can know for sure the data was received by the device
		// I believe Ezzence should support this, therefore is set to true.

		BluetoothLEHardwareInterface.WriteCharacteristic (_ezzence, ServiceUUID, Characteristic, data, data.Length, true, (characteristicUUID) => {
			BluetoothStatus.text = "Write String Succeeded";
			BluetoothLEHardwareInterface.Log ("Write String Succeeded");
		});
	}

	void SendByte (byte value)
	{
		byte[] data = new byte[] { value };

		BluetoothLEHardwareInterface.WriteCharacteristic (_ezzence, ServiceUUID, Characteristic, data, data.Length, true, (characteristicUUID) => {
			BluetoothStatus.text = "Write Byte Succeeded";
			BluetoothLEHardwareInterface.Log ("Write Byte Succeeded");
		});
	}

}
