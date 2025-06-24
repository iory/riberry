// --- Configuration ---
const GITHUB_REPO_OWNER = 'iory';
const GITHUB_REPO_NAME = 'riberry';
// --------------------

const installButton = document.getElementById('installButton');
const firmwareSelect = document.getElementById('firmwareSelect');
const deviceSelect = document.getElementById('deviceSelect');
const logDiv = document.getElementById('log');
const lcdPreview = document.getElementById('lcdPreview');
const lcdImage = document.getElementById('lcdImage');

let selectedFirmware = null;
let selectedDevice = 'atoms3';

function log(message, type = 'info') {
    const timestamp = new Date().toLocaleTimeString();
    const p = document.createElement('p');
    p.innerHTML = `<span class="text-gray-500">[${timestamp}]</span> ${message}`;

    switch (type) {
        case 'error': p.classList.add('text-red-400'); break;
        case 'success': p.classList.add('text-green-400'); break;
        case 'progress': p.classList.add('text-blue-400'); break;
        default: p.classList.add('text-gray-300');
    }

    logDiv.appendChild(p);
    logDiv.scrollTop = logDiv.scrollHeight;
}

function updateInstallButton() {
    // Remove ESP Web Tools button
    const oldButton = document.getElementById('installButton');
    if (oldButton) {
        oldButton.remove();
    }

    const container = document.querySelector('.flex.items-center.justify-center');

    if (selectedFirmware) {
        // Create new button
        const newButton = document.createElement('esp-web-install-button');
        newButton.id = 'installButton';
        newButton.setAttribute('manifest', generateManifestUrl(selectedFirmware));

        const activateButton = document.createElement('button');
        activateButton.slot = 'activate';
        activateButton.className = 'bg-green-600 hover:bg-green-700 text-white font-bold py-2 px-4 rounded-md transition-colors duration-200 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-offset-gray-800 focus:ring-green-500';
        activateButton.textContent = 'Flash Firmware';

        newButton.appendChild(activateButton);
        container.appendChild(newButton);

        log(`Install button updated for: ${selectedFirmware}`, 'success');
    } else {
        // Display disabled button
        const disabledButton = document.createElement('button');
        disabledButton.id = 'installButton';
        disabledButton.className = 'bg-gray-600 text-gray-400 font-bold py-2 px-4 rounded-md cursor-not-allowed';
        disabledButton.textContent = 'Please select firmware';
        disabledButton.disabled = true;

        container.appendChild(disabledButton);
    }
}

function generateManifestUrl(firmwareName) {
    const baseUrl = window.location.origin + window.location.pathname.replace(/\/[^/]*$/, '/');

    // Use raw GitHub URLs to bypass Jekyll processing
    const rawBaseUrl = 'https://raw.githubusercontent.com/iory/riberry/gh-pages/firmware/latest/';

    // Select bootloader and partitions files based on device type
    const bootloaderFile = selectedDevice === 'basic' ? 'm5stack-basic-bootloader.bin' : 'm5stack-atoms3-bootloader.bin';
    const partitionsFile = selectedDevice === 'basic' ? 'm5stack-basic-partitions.bin' : 'm5stack-atoms3-partitions.bin';

    const bootloaderUrl = `${rawBaseUrl}${bootloaderFile}`;
    const partitionsUrl = `${rawBaseUrl}${partitionsFile}`;
    const firmwareUrl = `${rawBaseUrl}${firmwareName}`;

    // Set chip family and device name based on device type
    const chipFamily = selectedDevice === 'basic' ? 'ESP32' : 'ESP32-S3';
    const deviceName = selectedDevice === 'basic' ? 'M5Stack Basic Firmware' : 'ATOM S3 Firmware';

    log(`Bootloader URL: ${bootloaderUrl}`, 'info');
    log(`Partitions URL: ${partitionsUrl}`, 'info');
    log(`Firmware URL: ${firmwareUrl}`, 'info');

    return `data:application/json;charset=utf-8,${encodeURIComponent(JSON.stringify({
        name: deviceName,
        version: "latest",
        builds: [{
            chipFamily: chipFamily,
            parts: [
                {
                    path: bootloaderUrl,
                    offset: 0x0000
                },
                {
                    path: partitionsUrl,
                    offset: 0x8000
                },
                {
                    path: firmwareUrl,
                    offset: 0x10000
                }
            ]
        }],
        "improv": false
    }, null, 2))}`;
}

/**
 * Fetches the contents of a specified directory from the GitHub API.
 * @param {string} path - The path of the directory to fetch.
 * @returns {Promise<Array>} - An array of directory/file information.
 */
async function fetchGithubDirListing(path) {
    const apiUrl = `https://api.github.com/repos/${GITHUB_REPO_OWNER}/${GITHUB_REPO_NAME}/contents/${path}?ref=gh-pages`;
    try {
        const response = await fetch(apiUrl);
        if (!response.ok) {
            // In case of 404, it's possible that the latest build does not exist yet or the directory name is wrong.
            if (response.status === 404) {
                 throw new Error(`Directory not found at '${path}'. Has the first build completed?`);
            }
            throw new Error(`GitHub API error! status: ${response.status}`);
        }
        return await response.json();
    } catch (error) {
        log(`Failed to fetch directory listing from GitHub: ${error.message}`, 'error');
        return [];
    }
}

/**
 * Firmware lists organized by device type
 */
const firmwareLists = {
    atoms3: [
        'm5stack-atoms3-lcd0-grove0.bin',
        'm5stack-atoms3-lcd0-grove1.bin',
        'm5stack-atoms3-lcd1-grove0.bin',
        'm5stack-atoms3-lcd1-grove1.bin',
        'm5stack-atoms3-lcd2-grove0.bin',
        'm5stack-atoms3-lcd2-grove1.bin',
        'm5stack-atoms3-lcd3-grove0.bin',
        'm5stack-atoms3-lcd3-grove1.bin'
    ],
    basic: [
        'm5stack-basic-lcd0-grove0.bin',
        'm5stack-basic-lcd1-grove0.bin',
        'm5stack-basic-lcd2-grove0.bin',
        'm5stack-basic-lcd3-grove0.bin'
    ]
};

/**
 * Loads the firmware variations for the selected device type
 */
function populateFirmwareForDevice(deviceType) {
    log(`Loading firmware list for ${deviceType}...`);

    const firmwareFiles = firmwareLists[deviceType] || [];

    firmwareSelect.innerHTML = '<option value="">-- Select a variation --</option>';

    firmwareFiles.forEach(fileName => {
        const option = document.createElement('option');
        option.value = fileName;
        option.textContent = fileName;
        firmwareSelect.appendChild(option);
    });

    log(`Firmware list loaded successfully for ${deviceType}.`, 'success');
}

/**
 * Loads the latest firmware variations and displays them in the dropdown.
 */
async function populateLatestFirmware() {
    populateFirmwareForDevice(selectedDevice);
}

/**
 * Validates if firmware files exist before enabling install
 */
async function validateFirmwareFiles(firmwareName) {
    const baseUrl = window.location.origin + window.location.pathname.replace(/\/[^/]*$/, '/');

    // Select bootloader and partitions files based on device type
    const bootloaderFile = selectedDevice === 'basic' ? 'm5stack-basic-bootloader.bin' : 'm5stack-atoms3-bootloader.bin';
    const partitionsFile = selectedDevice === 'basic' ? 'm5stack-basic-partitions.bin' : 'm5stack-atoms3-partitions.bin';

    const filesToCheck = [
        `${baseUrl}firmware/latest/${bootloaderFile}`,
        `${baseUrl}firmware/latest/${partitionsFile}`,
        `${baseUrl}firmware/latest/${firmwareName}`
    ];

    try {
        const results = await Promise.all(
            filesToCheck.map(async (url) => {
                try {
                    const response = await fetch(url, { method: 'HEAD' });
                    return response.ok;
                } catch {
                    return false;
                }
            })
        );

        return results.every(result => result);
    } catch {
        return false;
    }
}

/**
 * Extracts LCD number from firmware filename
 */
function extractLcdNumber(firmwareName) {
    const match = firmwareName.match(/lcd(\d+)/);
    return match ? match[1] : null;
}

/**
 * Updates the LCD preview image based on selected firmware
 */
function updateLcdPreview(firmwareName) {
    if (!firmwareName) {
        lcdPreview.classList.add('hidden');
        return;
    }

    const lcdNumber = extractLcdNumber(firmwareName);
    if (lcdNumber !== null) {
        const imagePath = `./imgs/atoms3-lcd${lcdNumber}.png`;
        lcdImage.src = imagePath;
        lcdImage.alt = `LCD${lcdNumber} Configuration`;
        lcdPreview.classList.remove('hidden');
        log(`Showing LCD${lcdNumber} configuration preview`, 'info');
    } else {
        lcdPreview.classList.add('hidden');
    }
}

/**
 * Handles device type selection change.
 */
function handleDeviceChange() {
    selectedDevice = deviceSelect.value;
    selectedFirmware = null;
    log(`Device type changed to: ${selectedDevice}`, 'info');
    populateFirmwareForDevice(selectedDevice);
    updateLcdPreview(null);
    updateInstallButton();
}

/**
 * Handles firmware selection change.
 */
async function handleFirmwareChange() {
    const firmwareName = firmwareSelect.value;

    if (firmwareName) {
        log(`Selected firmware: ${firmwareName} (skipping CORS validation)`, 'success');
        selectedFirmware = firmwareName;
        updateLcdPreview(firmwareName);
        updateInstallButton();
    } else {
        selectedFirmware = null;
        updateLcdPreview(null);
        updateInstallButton();
    }
}


// --- Event Listeners ---

// Directly fetch the list of the latest firmware on page load.
document.addEventListener('DOMContentLoaded', populateLatestFirmware);
// Execute the load process when a device type is selected.
deviceSelect.addEventListener('change', handleDeviceChange);
// Execute the load process when a firmware is selected.
firmwareSelect.addEventListener('change', handleFirmwareChange);

// Initialize install button state
updateInstallButton();
