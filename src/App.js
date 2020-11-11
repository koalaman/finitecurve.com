import React from 'react';
import Box from '@material-ui/core/Box';
import Button from '@material-ui/core/Button';
import Container from "@material-ui/core/Container";
import Checkbox from "@material-ui/core/Checkbox";
import Divider from "@material-ui/core/Divider";
import Drawer from "@material-ui/core/Drawer";
import FormControlLabel from "@material-ui/core/FormControlLabel";
import Grid from "@material-ui/core/Grid";
import { makeStyles } from '@material-ui/core/styles';
import { withStyles } from '@material-ui/core/styles';
import Slider from "@material-ui/core/Slider";
import Tooltip from "@material-ui/core/Tooltip";
import Typography from "@material-ui/core/Typography";
import CircularProgress from '@material-ui/core/CircularProgress';

import { MapInteractionCSS } from 'react-map-interaction';
import { ColorPicker } from 'material-ui-color';

import List from "@material-ui/core/List";
import ListItem from "@material-ui/core/ListItem";

import exPig from './examples/pig.jpg';
import exDog from './examples/dog.jpg';
import exWorld from './examples/world.png';

import OneLineClient from './OneLineClient.js';

import './App.css';
window.React = React;

const drawerWidth = 200;

const styles = {
  root: {
    display: 'flex',
    height: '100%',
  },
  fullscreen: {
    width: '100%',
    height: '100%',
  },
  drawer: {
    width: drawerWidth,
    flexShrink: 0,
  },
  drawerPaper: {
    width: drawerWidth,
    "overflow-x": "hidden",
  },
  // necessary for content to be below app bar
  // toolbar: theme.mixins.toolbar,
  content: {
    flexGrow: 1,
    position: "relative",
    overflow: "hidden",
    top: 0,
    bottom:0,
    left: 0,
    right: 0,
  },

  lowButton: {
    width: drawerWidth - 40,
    marginLeft: 20,
    marginRight: 20,
    marginTop: 20,
    marginBottom: 0,
  },

  highButton: {
    width: drawerWidth - 40,
    marginLeft: 20,
    marginRight: 20,
    marginTop: 0,
    marginBottom: 20,
  },

  lowHighButton: {
    width: drawerWidth - 40,
    marginLeft: 20,
    marginRight: 20,
    marginTop: 20,
    marginBottom: 20,
  },

  imageSelector: {
    display: "inline-block",
    padding: "25px",
    border: "solid 4px #aaa",
    "border-radius": "25px",
    backgroundColor: "#fff",
  },

  errorBox: {
    display: "inline-block",
    padding: "25px",
    border: "solid 4px #f00",
    "border-radius": "25px",
    backgroundColor: "#fff",
  },

  exampleBox: {
    width: "80px",
    height: "80px",
  },

  toast: {
    position: "absolute",
    top: "0.5em",
    left: "0.5em",
  },

  stats: {
    position: "absolute",
    bottom: "0.5em",
    left: "0.5em",
    fontSize: 12,
    color: "gray",
  },
};

const madeStyles = makeStyles(styles);

let handler = undefined;
let setMapSize = undefined;

const uiState = {
  SELECTING: 1,
  PROCESSING: 2,
  PENDING: 3,
  VIEWING: 4,
  ERROR: 5,
};


class App extends React.Component {
  constructor(props) {
    super(props);
    this.state = {
      lastDraw: 0,
      url: "data:",
      background: "white",
      width: 0,
      height: 0,
      ui: uiState.SELECTING,
      status: "",
      map: {
        scale: 1,
        translation: {
          x: 0,
          y: 0,
        }
      },
      controls: this.getDefaultControls(),
    };

    OneLineClient.onResult = d => this.processResult(this, d);
  }

  componentDidMount() {
    this.setImageUrl(process.env.PUBLIC_URL + "/splash.jpg", 1920, 1117);
  }

  getDefaultControls() {
      return {
        timestamp: Date.now(),
        resolution: 30,
        lineWidth: 4,
        contrast: 50,
        whiteCutoff: 240,
        invert: false,
        fg: "black",
        bg: "white",
      };
  }

  openFeedback() {
    window.location.href = "https://www.facebook.com/finitecurve";
  }

  getUiStateElement() {
    switch(this.state.ui) {
      case uiState.SELECTING:
        return (
          <ImageSelector
            onImageLoading={e => this.onImageLoading(e)}
            onImageSelected={e => this.onImageSelected(e)}
          />);
      case uiState.PROCESSING:
        return (
          <Spinner>Rendering...</Spinner>
        );
      case uiState.PENDING:
        return (
          <Spinner>Finishing previous...</Spinner>
        );
      case uiState.ERROR:
        return (
          <ErrorMessage onAccept={() => this.openImageSelection()}>
            {this.state.error}
          </ErrorMessage>
        );
      case uiState.VIEWING:
        return (<span />);
      default:
        alert("Developer messed up: " + this.state.ui);
    }
  }

  onImageLoading(event) {
    this.setStatus("Loading...");
  }

  onImageSelected(event) {
    OneLineClient.setImage(event.data);
    this.setStatus("Processing...");
    this.startBuild();
  }

  triggerBuild(lastTime) {
    if (lastTime < this.state.controls.timestamp) return;
    this.startBuild();
  }

  startBuild() {
    switch(this.state.ui) {
      case uiState.PROCESSING:
      case uiState.PENDING:
        this.setState({ui: uiState.PENDING });
        break;
      case uiState.VIEWING:
      case uiState.SELECTING:
        this.setState({ ui: uiState.PROCESSING }, () => OneLineClient.build(this.state.controls));
        break;
    }
  }

  setStatus(string) {
    this.setState({ status: string });
  }

  changeControls(c) {
    const time = Date.now();
    const next = { ...this.state.controls, ...c, timestamp: time };
    this.setState({ controls: next });
    if (this.state.ui !== uiState.SELECTING) {
      setTimeout(() => this.triggerBuild(time), 1000);
    }
  }

  openImageSelection() {
    this.setState({ ui: uiState.SELECTING });
  }

  downloadFile(name, url) {
    var element = document.createElement("a");
    element.setAttribute("href", url);
    element.setAttribute("download", name);
    document.body.appendChild(element);
    element.click();
    document.body.removeChild(element);
  }

  getPngUrl() {
    var canvas = document.createElement("canvas");
    var img = document.createElement("img");
    img.src = this.state.url;
    canvas.width = img.width = this.state.width;
    canvas.height = img.height = this.state.height;

    document.body.appendChild(canvas);
    document.body.appendChild(img);
    let ctx = canvas.getContext("2d");
    ctx.beginPath();
    ctx.rect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = this.toHexColor(this.state.controls.bg);
    ctx.fill();
    ctx.drawImage(img, 0, 0);
    let url = canvas.toDataURL('image/png');
    document.body.removeChild(img);
    document.body.removeChild(canvas);
    return url;
  }

  toHexColor(c) {
    if (typeof c === "string") {
      return c;
    }
    return "#" + c.hex;
  }

  render() {
    const { classes } = this.props;

    return (
      <div className={classes.root}>
        <AppDrawer {...this.state.controls}
            onChange={ c => this.changeControls(c) }
            onNewImage={() => this.openImageSelection()}
            onDownloadSVG={() => this.downloadFile("finitecurve.svg", this.state.url)}
            onDownloadPNG={() => this.downloadFile("finitecurve.png", this.getPngUrl())}
            onResetParams={() => this.changeControls(this.getDefaultControls())}
            onFeedback={() => this.openFeedback()}
            canSelect={this.state.ui !== uiState.SELECTING}
            canDownload={this.state.ui === uiState.VIEWING}
          />
        <div className={classes.content} id="content" style={{backgroundColor: this.state.background}}>
          <Typography className={classes.toast}>{this.getToastMessage()}</Typography>
          <Typography className={classes.stats}>{this.getStats()}</Typography>
          {this.getUiStateElement(this.state.ui)}
          <MapInteractionCSS value={this.state.map} onChange={(c) => this.setState({map: c})}>
            <img src={this.state.url} width={this.state.width + "px"} height={this.state.height + "px"}/>
          </MapInteractionCSS>
        </div>
      </div>
    );
  }

  getToastMessage() {
    if (this.state.ui !== uiState.VIEWING) return "";

    if (this.state.map.translation.x === 0 && this.state.map.translation.y === 0) {
      return "Pan/zoom to view details!";
    } else {
      return "";
    }
  }

  getStats() {
    if (this.state.ui !== uiState.VIEWING) return "";
    let dist = typeof(this.state.lineLength) === "number" ? this.state.lineLength.toFixed(0) : "?"
    return "Image: " + this.state.width + "x" + this.state.height + "px. Line: " + dist + "px";
  }

  swapForeground(svg, c) {
    const rep = "black";
    let index = svg.indexOf(rep);
    if (index === -1) {
      console.error("Oof, no color");
      return svg;
    }
    return svg.substring(0, index) + c + svg.substring(index+rep.length);
  }

  processResult(self, data) {
    if (data.success) {
      let url = "data:image/svg+xml;charset=utf-8;base64," + btoa(this.swapForeground(data.result, this.toHexColor(this.state.controls.fg)));
      this.setImageUrl(url, data.width, data.height, { lineLength: data.lineLength, background: this.toHexColor(this.state.controls.bg) })
    } else {
      this.setState({ ui: uiState.ERROR, error: data.error });
    }
  }

  setImageUrl(url, width, height, other) {
    const content = document.getElementById("content");
    const scale = Math.min(
      content.clientWidth / width,
      content.clientHeight / height);
    this.setState({
      url: url,
      width: width,
      height: height,
      map: {
        scale: scale,
        translation: {
          x: 0,
          y: 0,
        },
      },
      ...other
    });
    this.setStatus("");

    switch(this.state.ui) {
      case uiState.PROCESSING:
        this.setState({ ui: uiState.VIEWING });
        break;
      case uiState.PENDING:
        this.setState({ ui: uiState.VIEWING }, () => this.startBuild());
        break;
      default:
        break;
    }
  }
}

function removeKey(old) {
  var x = {...old};
  for(let i=1; i<arguments.length; i++) {
    delete x[arguments[i]];
  }
  return x;
}

function ParameterSlider(props) {
  return (
      <Grid container direction="column" spacing={0}>
        <Grid item>
          <Tooltip title={props.tooltip} arrow>
            <Typography>{props.title}</Typography>
          </Tooltip>
        </Grid>
        <Grid item>
          <Tooltip title={props.tooltip} arrow>
            <Slider
              min={props.min}
              max={props.max}
              value={props.value}
              onChange={(c, newValue) => props.onChange(newValue)}
              aria-labelledby="continuous-slider"
              valueLabelDisplay="auto"
              {...removeKey(props, "title")}
            />
          </Tooltip>
        </Grid>
      </Grid>
  );
}

function ParameterCheckbox(props) {
  return (
      <Grid container direction="column" spacing={0}>
        <Grid item>
          <Tooltip title={props.tooltip} arrow>
            <FormControlLabel control={
              <Checkbox
                color="primary"
                checked={props.value}
                onChange={(c, newValue) => props.onChange(newValue)}
                aria-labelledby="continuous-slider"
                valueLabelDisplay="auto"
                {...removeKey(props, "title")}
              />}
              label={props.title} />
          </Tooltip>
        </Grid>
      </Grid>
  );
}

function AppDrawer(props) {
  const classes = madeStyles();

  return (
    <Drawer variant="permanent" anchor="left" className={classes.drawer} classes={{paper: classes.drawerPaper}}>
      <List spacing={0}>
        <ListItem>
          <ParameterSlider min={0} max={100} value={props.resolution} onChange={(e, c) => props.onChange({ resolution: c })} title="Resolution" tooltip="Size of the result" />
        </ListItem>
        <ListItem>
          <ParameterSlider min={0.1} max={16} value={props.lineWidth} onChange={(e, c) => props.onChange({ lineWidth: c })} step={0.1} title="Stroke width" tooltip="How thick the line should be" />
        </ListItem>
        <ListItem>
          <ParameterSlider min={0} max={100} value={props.contrast} onChange={(e, c) => props.onChange({ contrast: c })} title="Contrast" tooltip="The difference in density between black and white areas" />
        </ListItem>
        <ListItem>
          <ParameterSlider min={0} max={255} value={props.whiteCutoff} onChange={(e, c) => props.onChange({ whiteCutoff: c })} title="White cutoff" tooltip="How white an area has to be to not draw in it" />
        </ListItem>
        <ListItem style={{marginTop: -10}}>
          <ParameterCheckbox value={props.invert} onChange={(e, c) => props.onChange({ invert: c })} title="Invert image" tooltip="Fill white instead of black" />
        </ListItem>
        <ListItem style={{marginTop: -10}}>
          Colors:
            <ColorPicker value={props.fg} hideTextfield disableAlpha onChange={c => props.onChange({ fg: c })} />
            <Button style={{ minWidth: 0 }} onClick={() => props.onChange( { fg: props.bg, bg: props.fg }) }>&#11020;</Button>
            <ColorPicker value={props.bg} hideTextfield disableAlpha onChange={c => props.onChange({ bg: c })} />
        </ListItem>
        <Button variant="contained" onClick={props.onResetParams} className={classes.highButton}>Reset</Button>
        <Divider />
        <Button variant="contained" color="primary" onClick={props.onNewImage} className={classes.lowButton} disabled={!props.canSelect}>Choose Image</Button>
        <Button variant="contained" color="primary" onClick={props.onDownloadSVG} className={classes.lowButton} disabled={!props.canDownload}>Download SVG</Button>
        <Button variant="contained" color="primary" onClick={props.onDownloadPNG} className={classes.lowHighButton} disabled={!props.canDownload}>Download PNG</Button>
        <Divider />
        <Button variant="contained" onClick={props.onFeedback} className={classes.lowButton}>Feedback (FB)</Button>
      </List>
    </Drawer>
  );
}

function loadFromImg(props, event) {
  let xhttp = new XMLHttpRequest();
  xhttp.responseType = "arraybuffer";
  xhttp.onreadystatechange = function() {
    if (this.readyState === 4) {
      if(this.status === 200) {
        onImageSelect(props, xhttp.response);
      } else {
        console.log("Can't load image", xhttp);
      }
    }
  };
  xhttp.open("GET", event.target.src, true);
  xhttp.send();
}

function loadFromFile(props, event) {
  if (event.target.files.length === 0) return;

  let reader = new FileReader();
  reader.onload = e => readerOnLoad(props, e);
  reader.onerror = clearImage;
  reader.readAsArrayBuffer(event.target.files[0]);
}

function readerOnLoad(props, event) {
  onImageSelect(props, event.target.result);
}

function onImageSelect(props, arrayBuffer) {
  props.onImageSelected({ data: arrayBuffer });
}

function clearImage() {
}

function ImageSelector(props) {
  const classes = madeStyles();
  const onClick = e => { props.onImageLoading(e); loadFromImg(props, e); }
  const onFile = e => { props.onImageLoading(e); loadFromFile(props, e); }
  return (
    <div id="imageSelector" style={{ "textAlign": "center", "width": "100%", "position": "absolute", "top": "50%", "transform": "translateY(-50%)", "zIndex": "1" }}>
    <div className={classes.imageSelector}>
      <div>
        <div style={{ display: "flex", "flexDirection": "column", "justifyContent": "center" }}>
          <Typography variant="h5">
            Try an example
          </Typography>
          <div style={{ "paddingTop": "20px", "paddingBottom": "20px" }}>
            <ImageExample onClick={onClick} src={exPig} title="Draw me like one of your French pigs. Oinque." />
            <ImageExample onClick={onClick} src={exWorld} title="Around the world, around the world, around the world, around the world - Daft Punk" />
            <ImageExample onClick={onClick} src={exDog} title="I've heard humans say it's a doggy dog world, and I couldn't agree more." />
          </div>
          <BorderWithText text="or" />
          <div>
            <Typography variant="h5" style={{"paddingTop": "20px", "paddingBottom": "20px" }}>
              Upload your own
            </Typography>
            <input type="file" id="file" onChange={onFile} accept="image/*" title="The image is processed locally and never uploaded. Do what you wish with this information." />
          </div>
        </div>
      </div>
    </div>
    </div>
  );
}

function Spinner(props) {
    const classes = madeStyles();
    return (
      <div style={{ "textAlign": "center", "width": "100%", "position": "absolute", "top": "50%", "transform": "translateY(-50%)", "zIndex": "1" }}>
        <div className={classes.imageSelector}>
          <CircularProgress color="primary" style={{ marginTop: 10, marginBottom: 10 }}/>
          <Typography>{props.children}</Typography>
        </div>
      </div>
    );
}

function ErrorMessage(props) {
    const classes = madeStyles();
    return (
      <div style={{ "textAlign": "center", "width": "100%", "position": "absolute", "top": "50%", "transform": "translateY(-50%)", "zIndex": "1" }}>
        <div className={classes.errorBox}>
          <Typography>{props.children}</Typography>
          <br />
          <Button variant="contained" color="primary" onClick={props.onAccept}>Ok</Button>
        </div>
      </div>
    );
}

function BorderWithText(props) {
  const borderStyle = { "borderBottom": "1px solid #aaa", "width": "100%", "height": "1px" };
  return (
    <div style={{"display": "flex", "alignItems": "center"}}>
      <div className="border" style={borderStyle} />
      <span style={{"marginLeft": "0.5em", "marginRight": "0.5em", "marginTop": "-0.1em" }}>{props.text}</span>
      <div className="border" style={borderStyle} />
    </div>
  );
}

function ImageExample(props) {
  return (
    <Button onClick={props.onClick} style={{"padding": "0px"}}>
      <img src={props.src} style={{"height": "100px"}} alt={props.title} title={props.title} />
    </Button>
  );
}

export default withStyles(styles)(App);
