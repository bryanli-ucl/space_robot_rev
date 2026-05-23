/*******************************************************************************
 * Copyright (c) 2014 IBM Corp.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * and the Eclipse Distribution License is available at
 *   http://www.eclipse.org/org/documents/edl-v10.php
 *******************************************************************************/

#ifndef MQTTUNSUBSCRIBE_H_
#define MQTTUNSUBSCRIBE_H_

#if defined(__cplusplus)
extern "C" {
#endif

int MQTTSerialize_unsubscribeLength(int count, MQTTString topicFilters[]);
int MQTTSerialize_unsubscribe(unsigned char* buf,
                              int buflen,
                              unsigned char dup,
                              unsigned short packetid,
                              int count,
                              MQTTString topicFilters[]);
int MQTTDeserialize_unsuback(unsigned short* packetid, unsigned char* buf, int buflen);

#if defined(__cplusplus)
}
#endif

#endif /* MQTTUNSUBSCRIBE_H_ */
