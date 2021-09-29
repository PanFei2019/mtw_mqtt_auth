/*
 * Copyright 2016-2021, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/*
 * Sample MCU application for BLE Battery Client (BATTC) profile using WICED HCI protocol.
 */


#include "app_include.h"

// Initialize app
void MainWindow::InitBATTC(void)
{

}

// connect to peer deivce
void MainWindow::on_btnBATTCConnect_clicked(void)
{
    BYTE    cmd[6];
    int     commandBytes = 0;

    if (m_CommPort == NULL)
        return;

    if (!m_bPortOpen)
    {
        return;
    }

    CBtDevice * pDev =(CBtDevice *)GetSelectedLEDevice() ;
    if (NULL == pDev)
        return;

    if(pDev->m_battc_handle != NULL_HANDLE)
    {
        Log("BATTC already connected for selected device");
        return;
    }

    for (int i = 0; i < 6; i++)
        cmd[commandBytes++] = pDev->m_address[5 - i];

    Log("BATTC Connect Command, BDA: %02x:%02x:%02x:%02x:%02x:%02x",
           pDev->m_address[0], pDev->m_address[1], pDev->m_address[2], pDev->m_address[3], pDev->m_address[4], pDev->m_address[5]);

    SendWicedCommand(HCI_CONTROL_BATT_CLIENT_COMMAND_CONNECT, cmd, 6);
}

// disconnect from peer deivce
void MainWindow::on_btnBATTCDisconnect_clicked(void)
{
    BYTE   cmd[60];
    int    commandBytes = 0;

    CBtDevice * pDev = GetConnectedBATTCDevice();
    if (pDev == NULL)
        return;
    USHORT nHandle = pDev->m_battc_handle;

    cmd[commandBytes++] = nHandle & 0xff;
    cmd[commandBytes++] = (nHandle >> 8) & 0xff;

    Log("BATTC Disconnect Command, Handle: %d", nHandle);
    SendWicedCommand(HCI_CONTROL_BATT_CLIENT_COMMAND_DISCONNECT, cmd, commandBytes);

    pDev->m_battc_handle = NULL_HANDLE;
    pDev->m_conn_type &= ~CONNECTION_TYPE_BATTC;
}

// disconnect from peer deivce
void MainWindow::on_btnBATTCReadLevel_clicked(void)
{
    BYTE   cmd[60];
    int    commandBytes = 0;

    CBtDevice * pDev = GetConnectedBATTCDevice();
    if (pDev == NULL)
        return;
    USHORT nHandle = pDev->m_battc_handle;

    cmd[commandBytes++] = nHandle & 0xff;
    cmd[commandBytes++] = (nHandle >> 8) & 0xff;

    Log("BATTC Read Level Command, Handle: %d", nHandle);
    SendWicedCommand(HCI_CONTROL_BATT_CLIENT_COMMAND_READ, cmd, commandBytes);
}

// Handle WICED HCI events
void MainWindow::onHandleWicedEventBATTC(unsigned int opcode, unsigned char *p_data, unsigned int len)
{
    switch (HCI_CONTROL_GROUP(opcode))
    {
    case HCI_CONTROL_GROUP_BATT_CLIENT:
        HandleBATTCHEvents(opcode, p_data, len);
        break;
    }
}

// Handle WICED HCI events for Battery Client
void MainWindow::HandleBATTCHEvents(DWORD opcode, LPBYTE p_data, DWORD len)
{
    char      trace[1024];
    BYTE       bda[6];
    CBtDevice *device;
    UINT16  handle;

    UNUSED(len);

    switch (opcode)
    {
    case HCI_CONTROL_BATT_CLIENT_EVENT_CONNECTED:
        for (int i = 0; i < 6; i++)
            bda[5 - i] = p_data[i + 1];
        sprintf(trace, "BATTC Connected status:%d address %02x:%02x:%02x:%02x:%02x:%02x handle:%d",
            p_data[0], bda[0], bda[1], bda[2], bda[3], bda[4], bda[5], p_data[7] + (p_data[8] << 8));
        Log(trace);
        handle = p_data[7] + (p_data[8] << 8);
        if (p_data[0] == 0)
        {
            if ((device = FindInList(bda,ui->cbBLEDeviceList)) == NULL)
                device = AddDeviceToList(bda, ui->cbBLEDeviceList, NULL);
            device->m_battc_handle = handle;
            device->m_conn_type |= CONNECTION_TYPE_BATTC;
            BattCDeviceAdd(bda);
            sprintf(trace, "BATTC Add Device address %02x:%02x:%02x:%02x:%02x:%02x",
                bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
            Log(trace);
            SendWicedCommand(HCI_CONTROL_BATT_CLIENT_COMMAND_ADD, &p_data[1], BD_ADDR_LEN);
        }
        SelectDevice(ui->cbBLEDeviceList, bda);
        break;

    case HCI_CONTROL_BATT_CLIENT_EVENT_DISCONNECTED:
        handle = p_data[0] | (p_data[1] << 8);
        sprintf(trace, "BATTC Connection Closed handle:%d reason:%d ", handle, p_data[2]);
        CBtDevice * pDev;

        pDev = FindInList(CONNECTION_TYPE_BATTC, handle, ui->cbBLEDeviceList);

        if (pDev && (pDev->m_battc_handle == handle))
        {
            pDev->m_battc_handle = NULL_HANDLE;
            pDev->m_conn_type &= ~CONNECTION_TYPE_BATTC;
        }
        Log(trace);
        break;

    case HCI_CONTROL_BATT_CLIENT_EVENT_LEVEL:
        sprintf(trace, "BATTC handle:%d status:%d level:%d", p_data[0] + (p_data[1] << 8), p_data[2], p_data[3]);
        Log(trace);
        break;

    case HCI_CONTROL_BATT_CLIENT_EVENT_STATUS:
        sprintf(trace, "BATTC Cmd Status:%d ", p_data[0]);
        Log(trace);
        break;

    default:
        sprintf(trace, "Rcvd Unknown BATTC OpCode: %d", (int)opcode);
        Log(trace);
        break;
    }
}

// Get selected device from BR/EDR combo box
CBtDevice* MainWindow::GetConnectedBATTCDevice(void)
{
    CBtDevice * pDev = (CBtDevice *)GetSelectedLEDevice();
    if (pDev == NULL)
    {
        Log("No device selected");
        return NULL;
    }

    if(pDev->m_battc_handle == NULL_HANDLE)
    {
        Log("Selected device is not connected as BATTC");
        return NULL;
    }

    return pDev;
}

void MainWindow::on_btnHelpBATTC_clicked()
{
    onClear();
    Log("Battery Client help topic:");
    Log("");
    Log("Apps : hci_ble_hid_host");
    Log("");
    Log("Connect to a BLE remote control or mouse that supports Battery service read battery level");
    ScrollToTop();
}
