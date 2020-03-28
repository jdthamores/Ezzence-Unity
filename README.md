# Ezzence <-> Unity Bridge

## **Communication between Ezzence BLE and Unity**

**The “EzzenceBasicBLE” Scene automatically connects to the Ezzence prototype and can release a burst of scent when the user presses the "Burst" button.**
*Note: This project has been tested in MacOS and iOS. When testing in "Play" mode, remember to add a new Aspect ratio for: iPhone 1334x750 Portrait 750x1334 and iPhone 1334x750 Landscape 1334x750 to correctly display the data.
*

### iOS Deployment:

* **Xcode Signing & Capabilities:** Automatically manage signing (fill out your Team, etc).

* **Xcode Info.plist:** This app needs BLE to connect to the Ezzence device, therefore the app's Info.plist must contain an *NSBluetoothAlwaysUsageDescription* key with a string value explaining to the user how the app uses this data.

* **Unity Project Settings:** Write a message for “Location usage Description”.

