package main

import (
	"net/http"
	"reflect"
	"testing"

	"github.com/aws/aws-lambda-go/events"
)

func TestPostHandlerIntegration(t *testing.T) {
	type request struct {
		request events.APIGatewayProxyRequest
	}
	testcases := []struct {
		name             string
		request          request
		expectedResponse events.APIGatewayProxyResponse
	}{
		{
			name: "Wrong http method",
			request: request{
				events.APIGatewayProxyRequest{
					HTTPMethod: "POT",
				},
			},
			expectedResponse: events.APIGatewayProxyResponse{
				StatusCode: http.StatusBadRequest,
				Body:       "400 Bad Request: Invalid http method",
			},
		},
		{
			name: "Incorrect Content-Type",
			request: request{
				events.APIGatewayProxyRequest{
					HTTPMethod: "POST",
					Headers: map[string]string{
						"Content-Type": "application/xml",
					},
				},
			},
			expectedResponse: events.APIGatewayProxyResponse{
				StatusCode: http.StatusBadRequest,
				Body:       "400 Bad Request: Invalid content type. It is not in a JSON format",
			},
		},
		{
			name: "Invalid request path",
			request: request{
				events.APIGatewayProxyRequest{
					Path:       "/api/d",
					HTTPMethod: "POST",
					Headers: map[string]string{
						"Content-Type": "application/json",
					},
				},
			},
			expectedResponse: events.APIGatewayProxyResponse{
				StatusCode: http.StatusForbidden,
				Body:       "403 Forbidden: You don't have permission to access /api/d",
			},
		},
		{
			name: "Request with empty body",
			request: request{
				events.APIGatewayProxyRequest{
					Path:       "/api/devices",
					HTTPMethod: "POST",
					Headers: map[string]string{
						"Content-Type": "application/json",
					},
					Body: "",
				},
			},
			expectedResponse: events.APIGatewayProxyResponse{
				StatusCode: http.StatusBadRequest,
				Body:       "400 Bad Request: Invalid JSON request. All fields are empty",
			},
		},
		{
			name: "Bad JSON format",
			request: request{
				events.APIGatewayProxyRequest{
					Path:       "/api/devices",
					HTTPMethod: "POST",
					Headers: map[string]string{
						"Content-Type": "application/json",
					},
					Body: "{{:\"id\":}\"/devices/id1\":{},\"deviceModel\":\"/devicemodels/id1\",\"name\":\"Sensor\",\"note\":\"Testing a sensor.\",\"serial\":\"A020000102\"}",
				},
			},
			expectedResponse: events.APIGatewayProxyResponse{
				StatusCode: http.StatusBadRequest,
				Body:       "400 Bad Request: Invalid JSON format",
			},
		},
		{
			"Request with missed field - ID",
			request{
				events.APIGatewayProxyRequest{
					Path:       "/api/devices",
					HTTPMethod: "POST",
					Headers: map[string]string{
						"Content-Type": "application/json",
					},
					Body: "{\"id\":\"\",\"deviceModel\":\"/devicemodels/id1\",\"name\":\"Sensor\",\"note\":\"Testing a sensor.\",\"serial\":\"A020000102\"}",
				},
			},
			events.APIGatewayProxyResponse{
				StatusCode: http.StatusBadRequest,
				Body:       "400 Bad Request: No value is specified for field(s) 'id' of device in json request",
			},
		},
		{
			"Request with missed field - DeviceModel",
			request{
				events.APIGatewayProxyRequest{
					Path:       "/api/devices",
					HTTPMethod: "POST",
					Headers: map[string]string{
						"Content-Type": "application/json",
					},
					Body: "{\"id\":\"/devices/id1\",\"deviceModel\":\"\",\"name\":\"Sensor\",\"note\":\"Testing a sensor.\",\"serial\":\"A020000102\"}",
				},
			},
			events.APIGatewayProxyResponse{
				StatusCode: http.StatusBadRequest,
				Body:       "400 Bad Request: No value is specified for field(s) 'device model' of device in json request",
			},
		},
		{
			"Request with missed field - Name",
			request{
				events.APIGatewayProxyRequest{
					Path:       "/api/devices",
					HTTPMethod: "POST",
					Headers: map[string]string{
						"Content-Type": "application/json",
					},
					Body: "{\"id\":\"/devices/id1\",\"deviceModel\":\"/devicemodels/id1\",\"name\":\"\",\"note\":\"Testing a sensor.\",\"serial\":\"A020000102\"}",
				},
			},
			events.APIGatewayProxyResponse{
				StatusCode: http.StatusBadRequest,
				Body:       "400 Bad Request: No value is specified for field(s) 'name' of device in json request",
			},
		},
		{
			"Request with missed field - Note",
			request{
				events.APIGatewayProxyRequest{
					Path:       "/api/devices",
					HTTPMethod: "POST",
					Headers: map[string]string{
						"Content-Type": "application/json",
					},
					Body: "{\"id\":\"/devices/id1\",\"deviceModel\":\"/devicemodels/id1\",\"name\":\"Sensor\",\"note\":\"\",\"serial\":\"A020000102\"}",
				},
			},
			events.APIGatewayProxyResponse{
				StatusCode: http.StatusBadRequest,
				Body:       "400 Bad Request: No value is specified for field(s) 'note' of device in json request",
			},
		},
		{
			"Request with missed field - Serial",
			request{
				events.APIGatewayProxyRequest{
					Path:       "/api/devices",
					HTTPMethod: "POST",
					Headers: map[string]string{
						"Content-Type": "application/json",
					},
					Body: "{\"id\":\"/devices/id1\",\"deviceModel\":\"/devicemodels/id1\",\"name\":\"Sensor\",\"note\":\"Testing a sensor.\",\"serial\":\"\"}",
				},
			},
			events.APIGatewayProxyResponse{
				StatusCode: http.StatusBadRequest,
				Body:       "400 Bad Request: No value is specified for field(s) 'serial' of device in json request",
			},
		},
		{
			"Request with multiple missed fields - Id, Name, Note",
			request{
				events.APIGatewayProxyRequest{
					Path:       "/api/devices",
					HTTPMethod: "POST",
					Headers: map[string]string{
						"Content-Type": "application/json",
					},
					Body: "{\"id\":\"\",\"deviceModel\":\"/devicemodels/id1\",\"name\":\"\",\"note\":\"\",\"serial\":\"A020000102\"}",
				},
			},
			events.APIGatewayProxyResponse{
				StatusCode: http.StatusBadRequest,
				Body:       "400 Bad Request: No value is specified for field(s) 'id' 'name' 'note' of device in json request",
			},
		},
		{
			"Complete request without error",
			request{
				events.APIGatewayProxyRequest{
					Path:       "/api/devices",
					HTTPMethod: "POST",
					Headers: map[string]string{
						"Content-Type": "application/json",
					},
					Body: "{\"id\":\"id2\",\"deviceModel\":\"/devicemodels/id2\",\"name\":\"Sensor\",\"note\":\"Testing a sensor.\",\"serial\":\"A020000102\"}",
				},
			},
			events.APIGatewayProxyResponse{
				StatusCode: http.StatusCreated,
				Body:       "{\"id\":\"id2\",\"deviceModel\":\"/devicemodels/id2\",\"name\":\"Sensor\",\"note\":\"Testing a sensor.\",\"serial\":\"A020000102\"}",
			},
		},
	}
	for _, testcase := range testcases {
		t.Run(testcase.name, func(t *testing.T) {
			if gotResponse := PostHandler(testcase.request.request); !reflect.DeepEqual(gotResponse, testcase.expectedResponse) {
				t.Errorf("%s \nGot output from postHandler() = %v\nExpected output = %v", testcase.name, gotResponse, testcase.expectedResponse)
			}
		})
	}
}
