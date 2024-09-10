/*
 * MIT License
 *
 * Copyright (c) 2021 touchgadgetdev@gmail.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// #include <Arduino.h>

#include <usb/usb_host.h>
// #include "show_desc.hpp"
#include "usbhhelp.hpp"

#include <Pushbutton.h>

bool isMIDI = false;
bool isMIDIReady = false;
uint8_t midiInterface = 0;

const size_t MIDI_IN_BUFFERS = 8;
const size_t MIDI_OUT_BUFFERS = 8;

usb_transfer_t *MIDIOut = NULL;
usb_transfer_t *MIDIIn[MIDI_IN_BUFFERS] = { NULL };

// CUSTOM SETUP
const unsigned char MIDI_OUTPUT_CHANNEL = 0xC0;  // C + TX Channel 0
Pushbutton decrementProgram(13);
Pushbutton incrementProgram(12);

// store current program. Is updated when PC message is read.
constexpr uint8_t minProgram = 0;
constexpr uint8_t maxProgram = 127;
static uint8_t currentProgram = minProgram;
static bool programChanged = false;
// END CUSTOM SETUP

static void ensure_current_program_bounds() {
  if (currentProgram < minProgram) {
    currentProgram = minProgram;
  }
  if (currentProgram > maxProgram) {
    currentProgram = maxProgram;
  }
}

// USB MIDI Event Packet Format (always 4 bytes)
//
// Byte 0 |Byte 1 |Byte 2 |Byte 3
// -------|-------|-------|------
// CN+CIN |MIDI_0 |MIDI_1 |MIDI_2
//
// CN = Cable Number (0x0..0xf) specifies virtual MIDI jack/cable
// CIN = Code Index Number (0x0..0xf) classifies the 3 MIDI bytes.
// See Table 4-1 in the MIDI 1.0 spec at usb.org.
static void midi_transfer_cb(usb_transfer_t *transfer) {
  ESP_LOGI("", "midi_transfer_cb context: %d", transfer->context);
  if (Device_Handle == transfer->device_handle) {
    int in_xfer = transfer->bEndpointAddress & USB_B_ENDPOINT_ADDRESS_EP_DIR_MASK;
    if ((transfer->status == 0) && in_xfer) {
      uint8_t *const data = transfer->data_buffer;
      for (int i = 0; i < transfer->actual_num_bytes; i += 4) {
        if ((data[i] + data[i + 1] + data[i + 2] + data[i + 3]) == 0) break;
        ESP_LOGI("", "midi: %02x %02x %02x %02x", data[i], data[i + 1], data[i + 2], data[i + 3]);

        // HANDLE PROGRAM CHANGE
        if (data[i + 1] == 0xc0) {
          ESP_LOGI("", "PC MESSAGE RECEIVED");
          currentProgram = data[i + 2];
          ensure_current_program_bounds();
          ESP_LOGI("", "Current Program: %d", currentProgram);
        }
      }

      esp_err_t err = usb_host_transfer_submit(transfer);
      if (err != ESP_OK) {
        ESP_LOGI("", "usb_host_transfer_submit In fail: %x", err);
      }
    } else {
      ESP_LOGI("", "transfer->status %d", transfer->status);
    }
  }
}

void check_interface_desc_MIDI(const void *p) {
  const usb_intf_desc_t *intf = (const usb_intf_desc_t *)p;

  // USB MIDI
  if ((intf->bInterfaceClass == USB_CLASS_AUDIO) && (intf->bInterfaceSubClass == 3) && (intf->bInterfaceProtocol == 0)) {
    isMIDI = true;
    ESP_LOGI("", "Claiming a MIDI device!");
    esp_err_t err = usb_host_interface_claim(
      Client_Handle, Device_Handle,
      intf->bInterfaceNumber, intf->bAlternateSetting);
    if (err != ESP_OK) {
      ESP_LOGI("", "usb_host_interface_claim failed: %x", err);
    } else {
      midiInterface = intf->bInterfaceNumber;
      ESP_LOGI("", "claimed interface: %d", midiInterface);
    }
  }
}

void prepare_endpoints(const void *p) {
  const usb_ep_desc_t *endpoint = (const usb_ep_desc_t *)p;
  esp_err_t err;

  ESP_LOGI("", "Preparing Endpoints...");
  // must be bulk or interrupt for MIDI
  if (!((endpoint->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK) == USB_BM_ATTRIBUTES_XFER_BULK || (endpoint->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK) == USB_BM_ATTRIBUTES_XFER_INT)) {
    ESP_LOGI("", "Not bulk or interrupt endpoint: 0x%02x", endpoint->bmAttributes);
    return;
  }
  if (endpoint->bEndpointAddress & USB_B_ENDPOINT_ADDRESS_EP_DIR_MASK) {
    ESP_LOGI("", "AAAAA");
    for (int i = 0; i < MIDI_IN_BUFFERS; i++) {
      err = usb_host_transfer_alloc(endpoint->wMaxPacketSize, 0, &MIDIIn[i]);
      if (err != ESP_OK) {
        MIDIIn[i] = NULL;
        ESP_LOGI("", "usb_host_transfer_alloc In fail: %x", err);
      } else {
        MIDIIn[i]->device_handle = Device_Handle;
        MIDIIn[i]->bEndpointAddress = endpoint->bEndpointAddress;
        MIDIIn[i]->callback = midi_transfer_cb;
        MIDIIn[i]->context = (void *)i;
        MIDIIn[i]->num_bytes = endpoint->wMaxPacketSize;
        esp_err_t err = usb_host_transfer_submit(MIDIIn[i]);
        if (err != ESP_OK) {
          ESP_LOGI("", "usb_host_transfer_submit In fail: %x", err);
        }
      }
    }
  } else {
    ESP_LOGI("", "BBBBB");
    err = usb_host_transfer_alloc(endpoint->wMaxPacketSize, 0, &MIDIOut);
    if (err != ESP_OK) {
      MIDIOut = NULL;
      ESP_LOGI("", "usb_host_transfer_alloc Out fail: %x", err);
      return;
    }
    ESP_LOGI("", "Out data_buffer_size: %d", MIDIOut->data_buffer_size);
    MIDIOut->device_handle = Device_Handle;
    MIDIOut->bEndpointAddress = endpoint->bEndpointAddress;
    MIDIOut->callback = midi_transfer_cb;
    MIDIOut->context = NULL;
    //    MIDIOut->flags |= USB_TRANSFER_FLAG_ZERO_PACK;
  }
  ESP_LOGI("", "Set isMIDIReady: %d && %d", (MIDIOut != NULL), (MIDIIn[0] != NULL));
  isMIDIReady = ((MIDIOut != NULL) && (MIDIIn[0] != NULL));
}

void show_config_desc_full(const usb_config_desc_t *config_desc) {
  // Full decode of config desc.
  const uint8_t *p = &config_desc->val[0];
  uint8_t bLength;
  for (int i = 0; i < config_desc->wTotalLength; i += bLength, p += bLength) {
    bLength = *p;
    if ((i + bLength) <= config_desc->wTotalLength) {
      const uint8_t bDescriptorType = *(p + 1);
      switch (bDescriptorType) {
        case USB_B_DESCRIPTOR_TYPE_DEVICE:
          ESP_LOGI("", "USB Device Descriptor should not appear in config");
          break;
        case USB_B_DESCRIPTOR_TYPE_CONFIGURATION:
          // show_config_desc(p);
          break;
        case USB_B_DESCRIPTOR_TYPE_STRING:
          ESP_LOGI("", "USB string desc TBD");
          break;
        case USB_B_DESCRIPTOR_TYPE_INTERFACE:
          // show_interface_desc(p);
          if (!isMIDI) check_interface_desc_MIDI(p);
          break;
        case USB_B_DESCRIPTOR_TYPE_ENDPOINT:
          // show_endpoint_desc(p);
          if (isMIDI && !isMIDIReady) {
            prepare_endpoints(p);
          } else {
            ESP_LOGE("", "NOT PREPARING ENDPOINTS!!!");
            ESP_LOGE("", "isMIDI: %d, !isMIDIReady: %d", isMIDI, !isMIDIReady);
          }
          break;
        case USB_B_DESCRIPTOR_TYPE_DEVICE_QUALIFIER:
          // Should not be in config?
          ESP_LOGI("", "USB device qual desc TBD");
          break;
        case USB_B_DESCRIPTOR_TYPE_OTHER_SPEED_CONFIGURATION:
          // Should not be in config?
          ESP_LOGI("", "USB Other Speed TBD");
          break;
        case USB_B_DESCRIPTOR_TYPE_INTERFACE_POWER:
          // Should not be in config?
          ESP_LOGI("", "USB Interface Power TBD");
          break;
        default:
          ESP_LOGI("", "Unknown USB Descriptor Type: 0x%x", *p);
          break;
      }
    } else {
      ESP_LOGI("", "USB Descriptor invalid");
      return;
    }
  }
}

void dev_gone() {
  esp_err_t err;
  isMIDI = false;
  isMIDIReady = false;

  for (int i = 0; i < MIDI_IN_BUFFERS; i++) {
    err = usb_host_transfer_free(MIDIIn[i]);
    ESP_LOGI("", "Midi IN transfer free: %x %s", err, esp_err_to_name(err));
    MIDIIn[i] = NULL;
  }

  err = usb_host_transfer_free(MIDIOut);
  if (err != ESP_OK) {
    ESP_LOGI("", "usb_host_transfer_free Out fail: %x", err);
  }
  ESP_LOGI("", "Midi OUT transfer free: %x %s", err, esp_err_to_name(err));
  MIDIOut = NULL;

  err = usb_host_interface_release(Client_Handle, Device_Handle, midiInterface);
  ESP_LOGI("", "release interface #%d: %x %s", midiInterface, err, esp_err_to_name(err));
  err = usb_host_device_close(Client_Handle, Device_Handle);
  ESP_LOGI("", "free device %x %s", err, esp_err_to_name(err));
}

void setup() {
  usbh_setup(show_config_desc_full, dev_gone);
}

void loop() {
  usbh_task();
  if (decrementProgram.getSingleDebouncedPress()) {
    if (currentProgram > minProgram) {
      currentProgram--;
      programChanged = true;
    }
  }
  if (incrementProgram.getSingleDebouncedPress()) {
    if (currentProgram < maxProgram) {
      currentProgram++;
      programChanged = true;
    }
  }

  if (programChanged) {
    unsigned char midiByte[4] = { 0x0C, 0xC0, currentProgram, 0x00 };

    ESP_LOGI("", "Writing Program: %d", currentProgram);
    if (isMIDIReady) {
      ESP_LOGI("", "MIDI send 4 bytes");
      MIDIOut->num_bytes = 4;
      memcpy(MIDIOut->data_buffer, midiByte, 4);
      esp_err_t err = usb_host_transfer_submit(MIDIOut);
      if (err != ESP_OK) {
        ESP_LOGI("", "usb_host_transfer_submit Out fail: %x", err);
      } else {
        programChanged = false;
      }
    }
  }
}